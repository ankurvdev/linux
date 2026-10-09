// SPDX-License-Identifier: GPL-2.0
/*
 * Qualcomm Core Voice Driver (CVD) client: voice call audio on the ADSP
 *
 * Call audio runs between the modem and the codec without passing through
 * Linux: the ADSP's voice services vocode it and route it to AFE ports.  This
 * client sets that up for a modem call:
 *
 *   MVM  (0x09)  Multimode Voice Manager - owns the session, starts/stops it
 *   CVS  (0x0A)  Core Voice Stream      - vocoder encode/decode
 *   CVP  (0x0B)  Core Voice Processor   - voice processing on the AFE ports
 *
 * The sessions are passive ones named after the VSID ("11C05000", VOICEMMODE1);
 * the modem's full-control session of the same name runs the state machine.
 *
 * A CS-VOICE front-end DAI exposes it to ALSA: its "Voice Mixer" switches pick
 * the AFE back ends, and opening its playback and capture PCMs starts them and
 * the session.  The PCMs carry no data.
 *
 * Copyright (c) 2026 Lance <Gero3977@gmail.com>
 * Copyright (c) 2026 Ankur Verma <ankur.verma@outlook.com>
 * CS-VOICE front end after Stephan Gerhold's msm8916 q6voice-dai driver.
 */

#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/soc/qcom/apr.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <dt-bindings/sound/qcom,q6dsp-lpass-ports.h>
#include <dt-bindings/sound/qcom,q6voice.h>

#define VSS_IMVM_CMD_CREATE_PASSIVE_CONTROL_SESSION	0x000110FF
#define VSS_IMVM_CMD_START_VOICE			0x00011190
#define VSS_IMVM_CMD_STOP_VOICE				0x00011192

#define VSS_ISTREAM_CMD_CREATE_PASSIVE_CONTROL_SESSION	0x00011140
#define VSS_IMVM_CMD_SET_POLICY_DUAL_CONTROL		0x00011327
#define VSS_IMVM_CMD_ATTACH_VOCPROC			0x0001123E
#define VSS_IMVM_CMD_DETACH_VOCPROC			0x0001123F
#define VSS_IVOCPROC_CMD_CREATE_FULL_CONTROL_SESSION_V2	0x000112BF
#define VSS_IVOCPROC_CMD_CREATE_FULL_CONTROL_SESSION_V3	0x00013169
#define VSS_IVOCPROC_CMD_TOPOLOGY_SET_DEV_CHANNELS	0x00013199
#define VSS_IVOCPROC_CMD_TOPOLOGY_COMMIT		0x00013198
#define VSS_ICOMMON_CMD_SET_PARAM_V2			0x0001133D
#define VSS_IVERSION_CMD_GET				0x00011378
#define VSS_IVERSION_RSP_GET				0x00011379
#define VSS_IVOCPROC_CMD_ENABLE				0x000100C6
#define VSS_IVOCPROC_CMD_DISABLE			0x000110E1
#define APRV2_IBASIC_CMD_DESTROY_SESSION		0x0001003C
#define VSS_IVOLUME_CMD_MUTE_V2				0x0001138B

#define VSS_IVOLUME_DIRECTION_TX			0
#define VSS_IVOLUME_DIRECTION_RX			1

#define VSS_IVOCPROC_DIRECTION_RX_TX			2
#define VSS_IVOCPROC_PORT_ID_NONE			0xFFFF
#define VSS_IVOCPROC_TOPOLOGY_ID_NONE			0x00010F70
#define VSS_IVOCPROC_TOPOLOGY_ID_RX_DEFAULT		0x00010F77

/* AFE port IDs.  The WCD codec - and so the earpiece and the call mic - is
 * behind SLIMbus port 0 on this board.
 */
#define AFE_PORT_ID_SLIMBUS_0_RX			0x4000
#define AFE_PORT_ID_QUATERNARY_MI2S_RX			0x1006
#define AFE_PORT_ID_QUATERNARY_MI2S_TX			0x1007
#define VSS_IVOCPROC_VOCPROC_MODE_EC_EXT_MIXING		0x00010F7D
#define VSS_PARAM_EC_REF_PORT_ENDPOINT_MEDIA_INFO	0x00013255
#define AFE_PORT_ID_SLIMBUS_0_TX			0x4001
#define VSS_IVOCPROC_VOCPROC_MODE_EC_INT_MIXING		0x00010F7C
#define VSS_ICOMMON_CAL_NETWORK_ID_NONE			0x0001135E
#define VSS_MODULE_CVD_GENERIC				0x0001316E
#define VSS_PARAM_TX_PORT_ENDPOINT_MEDIA_INFO		0x00013253
#define VSS_PARAM_RX_PORT_ENDPOINT_MEDIA_INFO		0x00013254
#define PCM_CHANNEL_FL					1
#define PCM_CHANNEL_FR					2
#define PCM_CHANNEL_FC					3

/* CVD versions, as the ADSP reports them, that change the vocproc setup */
#define CVD_VERSION_2_2					22
#define CVD_VERSION_2_3					23

#define SESSION_NAME_LEN		20
#define VOICEMMODE1_NAME		"11C05000"
#define Q6VOICE_TIMEOUT_MS		5000

struct vss_imvm_cmd_create_control_session {
	char name[SESSION_NAME_LEN];
} __packed;

struct vss_imvm_cmd_set_policy_dual_control {
	u8 enable;
} __packed;

struct vss_ivolume_cmd_mute_v2 {
	u16 direction;
	u16 mute_flag;
	u16 ramp_duration_ms;
} __packed;

struct vss_ivocproc_cmd_topology_set_dev_channels {
	u16 tx_num_channels;
	u16 rx_num_channels;
} __packed;

/* In-band VSS_ICOMMON_CMD_SET_PARAM_V2 carrying one parameter */
struct vss_set_param_media_format {
	u32 mem_handle;
	u64 mem_address;
	u32 payload_size;
	u32 module_id;
	u32 param_id;
	u16 param_size;
	u16 reserved;
	u32 port_id;
	u16 num_channels;
	u16 bits_per_sample;
	u32 sample_rate;
	u8 channel_mapping[8];
} __packed;

/* MVM attach commands carry a bare session handle. */
struct vss_cmd_attach_handle {
	u16 handle;
} __packed;

struct vss_ivocproc_cmd_create_full_control_session_v2 {
	u16 direction;
	u16 tx_port_id;
	u32 tx_topology_id;
	u16 rx_port_id;
	u32 rx_topology_id;
	u32 profile_id;
	u32 vocproc_mode;
	u16 ec_ref_port_id;
	char name[SESSION_NAME_LEN];
} __packed;

/* One APR service endpoint (MVM, CVS or CVP). */
struct q6voice_svc {
	struct apr_device *adev;
	wait_queue_head_t wait;

	/* The command in flight, and what its response reported. */
	u32 opcode;
	u32 token;
	bool resp_received;
	u32 status;
	u16 resp_port;

	/* Handle of the session this service holds, or 0 for none. */
	u16 handle;
};

static struct q6voice_svc q6voice_mvm;
static struct q6voice_svc q6voice_cvs;
static struct q6voice_svc q6voice_cvp;

/*
 * Serialises every command sequence against the others and against unbind:
 * building or tearing down a session takes several commands across all three
 * services, and none of them may be interleaved with another sequence.
 */
static DEFINE_MUTEX(q6voice_lock);

/* Whether a session has been started, so teardown can be idempotent. */
static bool q6voice_session_up;

/* AFE ports the vocproc is built against, chosen with the voice mixers */
static u16 rx_port = AFE_PORT_ID_SLIMBUS_0_RX;
static u16 tx_port = AFE_PORT_ID_SLIMBUS_0_TX;
/* External echo reference port (0 = none: internal EC mixing) */
static u16 ec_port;

/*
 * No calibration is loaded: Tx without processing, the default Rx chain.  The
 * vocproc Rx device is mono only; Tx carries one channel per routed codec port
 * (two digital mics).
 */
#define Q6VOICE_RX_TOPOLOGY	VSS_IVOCPROC_TOPOLOGY_ID_RX_DEFAULT
#define Q6VOICE_TX_TOPOLOGY	VSS_IVOCPROC_TOPOLOGY_ID_NONE
#define Q6VOICE_RX_CHANNELS	1
#define Q6VOICE_TX_CHANNELS	2

/* CVD version string reported by the ADSP ("2.3"), and as a number (23) */
static char q6voice_cvd_version[32];
static unsigned int q6voice_cvd;

static int q6voice_probe_svc(struct apr_device *adev, struct q6voice_svc *svc)
{
	guard(mutex)(&q6voice_lock);

	init_waitqueue_head(&svc->wait);
	svc->handle = 0;
	svc->adev = adev;
	dev_set_drvdata(&adev->dev, svc);

	return 0;
}

/*
 * Drop the binding before the bus tears the APR device down.  Taking the lock
 * waits out any command sequence still using the endpoint, and every later one
 * sees ->adev cleared and fails with -ENODEV.  Whatever session the ADSP still
 * holds for this service is lost with the binding.
 */
static void q6voice_remove_svc(struct q6voice_svc *svc)
{
	guard(mutex)(&q6voice_lock);

	svc->adev = NULL;
	svc->handle = 0;
	if (svc == &q6voice_mvm)
		q6voice_session_up = false;
}

/*
 * Every command is answered by APR_BASIC_RSP_RESULT, which echoes the command
 * opcode and the token it was sent with.  Anything else, or a result for some
 * other command, is not the answer being waited for.  For a create command,
 * the handle of the new session arrives as the source port of the response.
 */
static int q6voice_callback_svc(struct apr_device *adev,
				const struct apr_resp_pkt *data,
				struct q6voice_svc *svc)
{
	const struct aprv2_ibasic_rsp_result_t *result;

	if (data->hdr.opcode == VSS_IVERSION_RSP_GET &&
	    svc->opcode == VSS_IVERSION_CMD_GET) {
		size_t n = min_t(size_t, data->payload_size,
				 sizeof(q6voice_cvd_version) - 1);

		memcpy(q6voice_cvd_version, data->payload, n);
		q6voice_cvd_version[n] = '\0';
		svc->status = 0;
		svc->resp_received = true;
		wake_up(&svc->wait);
		return 0;
	}

	if (data->hdr.opcode != APR_BASIC_RSP_RESULT ||
	    data->payload_size < sizeof(*result)) {
		dev_dbg(&adev->dev, "unexpected opcode %#x, size %d\n",
			data->hdr.opcode, data->payload_size);
		return 0;
	}

	result = data->payload;
	if (result->opcode != svc->opcode || data->hdr.token != svc->token) {
		dev_dbg(&adev->dev, "stale result for %#x (token %#x)\n",
			result->opcode, data->hdr.token);
		return 0;
	}

	svc->status = result->status;
	svc->resp_port = data->hdr.src_port;
	svc->resp_received = true;
	wake_up(&svc->wait);

	return 0;
}

/*
 * Build and send one CVD command, and wait for its result.  @dest is the
 * handle of the session being addressed, or 0 when creating a session.
 * Called with q6voice_lock held.
 */
static int q6voice_cmd(struct q6voice_svc *svc, u32 opcode, u16 dest,
		       const void *payload, size_t payload_size)
{
	size_t pkt_size = APR_HDR_SIZE + payload_size;
	struct apr_pkt *pkt;
	int ret;

	lockdep_assert_held(&q6voice_lock);

	if (!svc->adev)
		return -ENODEV;

	void *p __free(kfree) = kzalloc(pkt_size, GFP_KERNEL);
	if (!p)
		return -ENOMEM;

	svc->opcode = opcode;
	svc->token++;
	svc->resp_received = false;

	pkt = p;
	pkt->hdr.hdr_field = APR_SEQ_CMD_HDR_FIELD;
	pkt->hdr.pkt_size = pkt_size;
	pkt->hdr.src_port = 0;
	pkt->hdr.dest_port = dest;
	pkt->hdr.token = svc->token;
	pkt->hdr.opcode = opcode;

	if (payload_size)
		memcpy(p + APR_HDR_SIZE, payload, payload_size);

	ret = apr_send_pkt(svc->adev, pkt);
	if (ret < 0) {
		dev_err(&svc->adev->dev, "failed to send %#x: %d\n", opcode, ret);
		return ret;
	}

	ret = wait_event_timeout(svc->wait, svc->resp_received,
				 msecs_to_jiffies(Q6VOICE_TIMEOUT_MS));
	if (!ret) {
		dev_err(&svc->adev->dev, "timeout waiting for %#x\n", opcode);
		return -ETIMEDOUT;
	}

	if (svc->status) {
		dev_err(&svc->adev->dev, "%#x failed: ADSP error %#x\n",
			opcode, svc->status);
		return -EINVAL;
	}

	return 0;
}

static int q6voice_create(struct q6voice_svc *svc, u32 opcode,
			  const void *payload, size_t payload_size)
{
	int ret;

	ret = q6voice_cmd(svc, opcode, 0, payload, payload_size);
	if (ret)
		return ret;

	svc->handle = svc->resp_port;
	dev_dbg(&svc->adev->dev, "session created, handle %#06x\n", svc->handle);

	return 0;
}

static int q6voice_create_session(struct q6voice_svc *svc, u32 opcode,
				  const char *name)
{
	struct vss_imvm_cmd_create_control_session session = {};

	strscpy(session.name, name, sizeof(session.name));

	return q6voice_create(svc, opcode, &session, sizeof(session));
}

static int q6voice_attach(struct q6voice_svc *svc, u32 opcode, u16 dest,
			  u16 handle)
{
	struct vss_cmd_attach_handle attach = { .handle = handle };

	return q6voice_cmd(svc, opcode, dest, &attach, sizeof(attach));
}

/*
 * Build the vocproc against the AFE ports the board routes voice over.  The
 * ADSP will happily create a vocproc with no ports but refuses to enable one,
 * so the ports have to be real even before any audio flows through them.
 */
static int q6voice_create_vocproc(const char *name)
{
	struct vss_ivocproc_cmd_create_full_control_session_v2 cvp = {
		.direction	= VSS_IVOCPROC_DIRECTION_RX_TX,
		.tx_port_id	= tx_port,
		.tx_topology_id	= Q6VOICE_TX_TOPOLOGY,
		.rx_port_id	= rx_port,
		.rx_topology_id	= Q6VOICE_RX_TOPOLOGY,
		.profile_id	= VSS_ICOMMON_CAL_NETWORK_ID_NONE,
		.vocproc_mode	= ec_port ? VSS_IVOCPROC_VOCPROC_MODE_EC_EXT_MIXING :
					  VSS_IVOCPROC_VOCPROC_MODE_EC_INT_MIXING,
		.ec_ref_port_id	= ec_port ?: VSS_IVOCPROC_PORT_ID_NONE,
	};

	strscpy(cvp.name, name, sizeof(cvp.name));

	return q6voice_create(&q6voice_cvp, q6voice_cvd >= CVD_VERSION_2_2 ?
			      VSS_IVOCPROC_CMD_CREATE_FULL_CONTROL_SESSION_V3 :
			      VSS_IVOCPROC_CMD_CREATE_FULL_CONTROL_SESSION_V2,
			      &cvp, sizeof(cvp));
}

static int q6voice_set_port_format(u32 param_id, u16 port, u16 channels)
{
	struct vss_set_param_media_format p = {
		.payload_size	= sizeof(p) - offsetof(typeof(p), module_id),
		.module_id	= VSS_MODULE_CVD_GENERIC,
		.param_id	= param_id,
		.param_size	= sizeof(p) - offsetof(typeof(p), port_id),
		.port_id	= port,
		.num_channels	= channels,
		.bits_per_sample = 16,
		.sample_rate	= 48000,
	};

	if (channels == 1) {
		p.channel_mapping[0] = PCM_CHANNEL_FC;
	} else {
		p.channel_mapping[0] = PCM_CHANNEL_FL;
		p.channel_mapping[1] = PCM_CHANNEL_FR;
	}

	return q6voice_cmd(&q6voice_cvp, VSS_ICOMMON_CMD_SET_PARAM_V2,
			   q6voice_cvp.handle, &p, sizeof(p));
}

/*
 * Tell the vocproc what the AFE ports carry (downstream
 * voice_send_cvp_media_fmt_info_cmd() and topology commit): the SLIMbus back
 * ends run at 48 kHz, 16 bit, one channel per routed codec port.
 */
static int q6voice_setup_vocproc(void)
{
	struct vss_ivocproc_cmd_topology_set_dev_channels ch = {
		.tx_num_channels = Q6VOICE_TX_CHANNELS,
		.rx_num_channels = Q6VOICE_RX_CHANNELS,
	};
	int ret;

	if (q6voice_cvd < CVD_VERSION_2_2)
		return 0;

	ret = q6voice_cmd(&q6voice_cvp, VSS_IVOCPROC_CMD_TOPOLOGY_SET_DEV_CHANNELS,
			  q6voice_cvp.handle, &ch, sizeof(ch));
	if (ret)
		return ret;

	if (q6voice_cvd >= CVD_VERSION_2_3) {
		ret = q6voice_set_port_format(VSS_PARAM_RX_PORT_ENDPOINT_MEDIA_INFO,
					      rx_port, Q6VOICE_RX_CHANNELS);
		if (ret)
			return ret;
		ret = q6voice_set_port_format(VSS_PARAM_TX_PORT_ENDPOINT_MEDIA_INFO,
					      tx_port, Q6VOICE_TX_CHANNELS);
		if (ret)
			return ret;
		/* The echo reference port runs mono (see the machine driver) */
		if (ec_port) {
			ret = q6voice_set_port_format(VSS_PARAM_EC_REF_PORT_ENDPOINT_MEDIA_INFO,
						      ec_port, 1);
			if (ret)
				return ret;
		}
	}

	return q6voice_cmd(&q6voice_cvp, VSS_IVOCPROC_CMD_TOPOLOGY_COMMIT,
			   q6voice_cvp.handle, NULL, 0);
}

static void q6voice_get_version(void)
{
	unsigned int major = 0, minor = 0;

	if (q6voice_cvd)
		return;

	q6voice_cvd_version[0] = '\0';
	if (q6voice_cmd(&q6voice_mvm, VSS_IVERSION_CMD_GET, 0, NULL, 0) ||
	    sscanf(q6voice_cvd_version, "%u.%u", &major, &minor) != 2)
		major = minor = 0;
	q6voice_cvd = major * 10 + minor;
	dev_dbg(&q6voice_mvm.adev->dev, "CVD version '%s'\n",
		 q6voice_cvd_version);
}

/*
 * Bring up a voice session end to end, in the order the downstream driver uses
 * for a modem call (voc_start_voice_call()).  The sessions are passive: the
 * modem's own full-control session of the same name owns the state machine and
 * attaches the stream itself (attaching it again fails with EALREADY), and
 * dual control lets the modem start and stop the voice path with the call.
 */
static int q6voice_build_session(const char *name)
{
	struct vss_imvm_cmd_set_policy_dual_control dual = { .enable = 1 };
	struct vss_ivolume_cmd_mute_v2 unmute = {
		.direction = VSS_IVOLUME_DIRECTION_TX,
		.ramp_duration_ms = 20,
	};
	int ret;

	q6voice_get_version();

	ret = q6voice_create_session(&q6voice_mvm,
				     VSS_IMVM_CMD_CREATE_PASSIVE_CONTROL_SESSION,
				     name);
	if (ret)
		return ret;

	ret = q6voice_create_session(&q6voice_cvs,
				     VSS_ISTREAM_CMD_CREATE_PASSIVE_CONTROL_SESSION,
				     name);
	if (ret)
		return ret;

	ret = q6voice_cmd(&q6voice_mvm, VSS_IMVM_CMD_SET_POLICY_DUAL_CONTROL,
			  q6voice_mvm.handle, &dual, sizeof(dual));
	if (ret)
		return ret;

	ret = q6voice_create_vocproc(name);
	if (ret)
		return ret;

	ret = q6voice_setup_vocproc();
	if (ret)
		return ret;

	ret = q6voice_cmd(&q6voice_cvp, VSS_IVOCPROC_CMD_ENABLE,
			  q6voice_cvp.handle, NULL, 0);
	if (ret)
		return ret;

	ret = q6voice_attach(&q6voice_mvm, VSS_IMVM_CMD_ATTACH_VOCPROC,
			     q6voice_mvm.handle, q6voice_cvp.handle);
	if (ret)
		return ret;

	/* Unmute the Tx stream and the Rx device; not fatal, as downstream */
	if (q6voice_cmd(&q6voice_cvs, VSS_IVOLUME_CMD_MUTE_V2,
			q6voice_cvs.handle, &unmute, sizeof(unmute)))
		dev_warn(&q6voice_cvs.adev->dev, "Tx unmute failed\n");
	unmute.direction = VSS_IVOLUME_DIRECTION_RX;
	if (q6voice_cmd(&q6voice_cvp, VSS_IVOLUME_CMD_MUTE_V2,
			q6voice_cvp.handle, &unmute, sizeof(unmute)))
		dev_warn(&q6voice_cvp.adev->dev, "Rx unmute failed\n");

	ret = q6voice_cmd(&q6voice_mvm, VSS_IMVM_CMD_START_VOICE,
			  q6voice_mvm.handle, NULL, 0);
	if (ret)
		return ret;

	q6voice_session_up = true;
	dev_dbg(&q6voice_mvm.adev->dev, "voice session started\n");

	return 0;
}

/*
 * Tear the session down in the exact reverse of the order it was built.  The
 * ADSP keeps whatever it was given: a session that is not destroyed survives
 * module unload, and a later create for the same VSID then goes unanswered
 * rather than being refused - which presents as a timeout on a command that
 * worked a moment earlier.
 *
 * Every step is attempted even if an earlier one fails; leaving an object
 * behind is worse than a failed command, because it is what poisons the next
 * create.
 */
static int q6voice_stop_voice(void)
{
	int ret, err = 0;

	lockdep_assert_held(&q6voice_lock);

	if (!q6voice_mvm.adev)
		return -ENODEV;

	if (!q6voice_session_up && !q6voice_mvm.handle &&
	    !q6voice_cvs.handle && !q6voice_cvp.handle)
		return 0;

	if (q6voice_session_up && q6voice_mvm.handle) {
		ret = q6voice_cmd(&q6voice_mvm, VSS_IMVM_CMD_STOP_VOICE,
				  q6voice_mvm.handle, NULL, 0);
		if (ret)
			err = ret;
	}

	if (q6voice_mvm.handle && q6voice_cvp.handle) {
		ret = q6voice_attach(&q6voice_mvm, VSS_IMVM_CMD_DETACH_VOCPROC,
				     q6voice_mvm.handle, q6voice_cvp.handle);
		if (ret)
			err = ret;
	}

	if (q6voice_cvp.handle) {
		ret = q6voice_cmd(&q6voice_cvp, VSS_IVOCPROC_CMD_DISABLE,
				  q6voice_cvp.handle, NULL, 0);
		if (ret)
			err = ret;

		ret = q6voice_cmd(&q6voice_cvp, APRV2_IBASIC_CMD_DESTROY_SESSION,
				  q6voice_cvp.handle, NULL, 0);
		if (ret)
			err = ret;
		else
			q6voice_cvp.handle = 0;
	}

	if (q6voice_cvs.handle) {
		ret = q6voice_cmd(&q6voice_cvs, APRV2_IBASIC_CMD_DESTROY_SESSION,
				  q6voice_cvs.handle, NULL, 0);
		if (ret)
			err = ret;
		else
			q6voice_cvs.handle = 0;
	}

	if (q6voice_mvm.handle) {
		ret = q6voice_cmd(&q6voice_mvm, APRV2_IBASIC_CMD_DESTROY_SESSION,
				  q6voice_mvm.handle, NULL, 0);
		if (ret)
			err = ret;
		else
			q6voice_mvm.handle = 0;
	}

	q6voice_session_up = false;

	dev_dbg(&q6voice_mvm.adev->dev, "voice session torn down%s\n",
		 err ? " (with errors)" : "");

	return err;
}

static int q6voice_start_voice(const char *name)
{
	int ret;

	lockdep_assert_held(&q6voice_lock);

	if (!q6voice_mvm.adev || !q6voice_cvs.adev || !q6voice_cvp.adev)
		return -ENODEV;

	if (q6voice_session_up)
		return -EBUSY;

	ret = q6voice_build_session(name);
	if (ret) {
		/*
		 * Whatever was created before the failure still exists on the
		 * ADSP and would poison the next attempt, so unwind it here.
		 */
		q6voice_stop_voice();
		return ret;
	}

	return 0;
}


/*
 * CS-VOICE front end.  Opening its playback and capture PCMs starts the AFE
 * back ends routed to it with the "Voice Mixer" switches -- for voice alone,
 * with no ADM stream on the port -- and, once both are prepared, the voice
 * session on those ports.  No audio data passes through the PCMs themselves.
 */
static unsigned int q6voice_dai_active;

static int q6voice_dai_prepare(struct snd_pcm_substream *substream,
			       struct snd_soc_dai *dai)
{
	int ret = 0;

	guard(mutex)(&q6voice_lock);

	dev_dbg(dai->dev, "prepare %s\n", substream->stream ? "capture" : "playback");
	q6voice_dai_active |= BIT(substream->stream);
	if (q6voice_dai_active == 3 && !q6voice_session_up)
		ret = q6voice_start_voice(VOICEMMODE1_NAME);

	return ret;
}

/*
 * Stop the session in hw_free: DPCM frees the front end before the back
 * ends, so the vocproc is gone before its AFE ports stop.  Stopping the
 * ports under a running vocproc left SLIMBUS_0_TX refusing to start again
 * (AFE error 1) and then hung the system.
 */
static int q6voice_dai_hw_free(struct snd_pcm_substream *substream,
			       struct snd_soc_dai *dai)
{
	guard(mutex)(&q6voice_lock);

	q6voice_dai_active &= ~BIT(substream->stream);
	if (q6voice_session_up || q6voice_mvm.handle)
		q6voice_stop_voice();
	return 0;
}

static void q6voice_dai_shutdown(struct snd_pcm_substream *substream,
				 struct snd_soc_dai *dai)
{
	q6voice_dai_hw_free(substream, dai);
}

static const struct snd_soc_dai_ops q6voice_dai_ops = {
	.prepare = q6voice_dai_prepare,
	.hw_free = q6voice_dai_hw_free,
	.shutdown = q6voice_dai_shutdown,
};

static struct snd_soc_dai_driver q6voice_dais[] = {
	{
		.id = CS_VOICE,
		.name = "CS-VOICE",
		.playback = {
			.stream_name = "CS-VOICE Playback",
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.rates = SNDRV_PCM_RATE_8000,
			.rate_min = 8000,
			.rate_max = 8000,
			.channels_min = 1,
			.channels_max = 1,
		},
		.capture = {
			.stream_name = "CS-VOICE Capture",
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.rates = SNDRV_PCM_RATE_8000,
			.rate_min = 8000,
			.rate_max = 8000,
			.channels_min = 1,
			.channels_max = 1,
		},
		.ops = &q6voice_dai_ops,
	},
};

static const struct snd_pcm_hardware q6voice_dai_hardware = {
	.info = SNDRV_PCM_INFO_INTERLEAVED,
	.buffer_bytes_max = 4096 * 2,
	.period_bytes_min = 2048,
	.period_bytes_max = 4096,
	.periods_min = 2,
	.periods_max = 4,
};


/*
 * Voice data never passes through the PCM; give it a buffer nobody reads so
 * ordinary ALSA clients can open, start and keep it running.
 */
static int q6voice_pcm_new(struct snd_soc_component *component,
			   struct snd_soc_pcm_runtime *rtd)
{
	snd_pcm_set_managed_buffer_all(rtd->pcm, SNDRV_DMA_TYPE_CONTINUOUS,
				       NULL, 0, q6voice_dai_hardware.buffer_bytes_max);
	return 0;
}

/*
 * A work item moves the PCM position along at the stream rate, so clients see a
 * running stream (and keep it, and the voice path, open) for the whole call.
 * The DPCM links are nonatomic (their stream lock is a mutex), so this must run
 * in process context: an hrtimer callback taking that lock deadlocked against
 * a client stopping the stream.
 */
struct q6voice_pcm {
	struct delayed_work work;
	struct snd_pcm_substream *substream;
	snd_pcm_uframes_t pos;
	unsigned long period_jiffies;
	bool running;
};

static void q6voice_pcm_tick(struct work_struct *w)
{
	struct q6voice_pcm *vp = container_of(to_delayed_work(w),
					      struct q6voice_pcm, work);
	struct snd_pcm_substream *ss = vp->substream;
	struct snd_pcm_runtime *rt = ss->runtime;
	snd_pcm_sframes_t room;

	if (!READ_ONCE(vp->running))
		return;

	/*
	 * Never run past the client: an xrun stops the stream, and DPCM then
	 * stops the back ends (and the call audio) with it.  Advance at most to
	 * one frame short of what has been written (playback), or of a full
	 * buffer of unread frames (capture).
	 */
	if (ss->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		/* frames written but not yet "played" */
		room = (snd_pcm_sframes_t)rt->control->appl_ptr -
		       (snd_pcm_sframes_t)rt->status->hw_ptr;
		if (room < 0)
			room += rt->boundary;
	} else {
		/* free space: buffer minus frames captured but not yet read */
		room = (snd_pcm_sframes_t)rt->status->hw_ptr -
		       (snd_pcm_sframes_t)rt->control->appl_ptr;
		if (room < 0)
			room += rt->boundary;
		room = rt->buffer_size - room;
	}
	room--;

	vp->pos = (vp->pos + clamp_t(snd_pcm_sframes_t, room, 0,
				     rt->period_size)) % rt->buffer_size;
	snd_pcm_period_elapsed(ss);

	if (READ_ONCE(vp->running))
		schedule_delayed_work(&vp->work, vp->period_jiffies);
}

static int q6voice_pcm_trigger(struct snd_soc_component *component,
			       struct snd_pcm_substream *substream, int cmd)
{
	struct q6voice_pcm *vp = substream->runtime->private_data;
	struct snd_pcm_runtime *rt = substream->runtime;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		vp->period_jiffies = max(1UL, msecs_to_jiffies(
			div_u64((u64)rt->period_size * MSEC_PER_SEC, rt->rate)));
		WRITE_ONCE(vp->running, true);
		schedule_delayed_work(&vp->work, vp->period_jiffies);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		/* Called with the stream lock held: don't wait for the work */
		WRITE_ONCE(vp->running, false);
		cancel_delayed_work(&vp->work);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static snd_pcm_uframes_t q6voice_pcm_pointer(struct snd_soc_component *component,
					     struct snd_pcm_substream *substream)
{
	struct q6voice_pcm *vp = substream->runtime->private_data;

	return vp->pos;
}

static int q6voice_pcm_close(struct snd_soc_component *component,
			     struct snd_pcm_substream *substream)
{
	struct q6voice_pcm *vp = substream->runtime->private_data;

	WRITE_ONCE(vp->running, false);
	cancel_delayed_work_sync(&vp->work);
	kfree(vp);
	return 0;
}

static int q6voice_pcm_open(struct snd_soc_component *component,
			    struct snd_pcm_substream *substream)
{
	struct q6voice_pcm *vp = kzalloc(sizeof(*vp), GFP_KERNEL);

	if (!vp)
		return -ENOMEM;
	INIT_DELAYED_WORK(&vp->work, q6voice_pcm_tick);
	vp->substream = substream;
	substream->runtime->private_data = vp;
	substream->runtime->hw = q6voice_dai_hardware;
	return 0;
}

static unsigned int q6voice_reg_read(struct snd_soc_component *component,
				     unsigned int reg)
{
	return 0;
}

static int q6voice_reg_write(struct snd_soc_component *component,
			     unsigned int reg, unsigned int val)
{
	return 0;
}

/* Mixer switch: reg = AFE port ID; shift 0 = Rx, 1 = Tx (mic), 2 = echo reference */
static int q6voice_mixer_get(struct snd_kcontrol *kc,
			     struct snd_ctl_elem_value *uc)
{
	struct soc_mixer_control *mc = (void *)kc->private_value;

	uc->value.integer.value[0] =
		(mc->shift == 2 ? ec_port : mc->shift ? tx_port : rx_port) == mc->reg;
	return 0;
}

static int q6voice_mixer_put(struct snd_kcontrol *kc,
			     struct snd_ctl_elem_value *uc)
{
	struct snd_soc_dapm_context *dapm = snd_soc_dapm_kcontrol_to_dapm(kc);
	struct soc_mixer_control *mc = (void *)kc->private_value;
	bool on = !!uc->value.integer.value[0];
	u16 *port = mc->shift == 2 ? &ec_port : mc->shift ? &tx_port : &rx_port;

	/*
	 * Route changes during a call (earpiece <-> speaker): stop the session
	 * before its port is disconnected, and start it again once the new
	 * back end is connected.
	 */
	if (!on && *port == mc->reg) {
		guard(mutex)(&q6voice_lock);
		if (q6voice_session_up)
			q6voice_stop_voice();
	}

	if (on)
		*port = mc->reg;
	else if (mc->shift == 2 && *port == mc->reg)
		*port = 0;
	snd_soc_dapm_mixer_update_power(dapm, kc, on, NULL);

	if (on) {
		guard(mutex)(&q6voice_lock);
		if (q6voice_session_up)
			q6voice_stop_voice();
		if (q6voice_dai_active == 3)
			q6voice_start_voice(VOICEMMODE1_NAME);
	}
	return 1;
}

static const struct snd_kcontrol_new q6voice_tx_mixer[] = {
	SOC_SINGLE_EXT("SLIMBUS_0_TX", AFE_PORT_ID_SLIMBUS_0_TX, 1, 1, 0,
		       q6voice_mixer_get, q6voice_mixer_put),
};

static const struct snd_kcontrol_new q6voice_ec_mixer[] = {
	SOC_SINGLE_EXT("QUAT_MI2S_TX", AFE_PORT_ID_QUATERNARY_MI2S_TX, 2, 1, 0,
		       q6voice_mixer_get, q6voice_mixer_put),
};

static const struct snd_kcontrol_new q6voice_slim0_rx_mixer[] = {
	SOC_SINGLE_EXT("CS-Voice", AFE_PORT_ID_SLIMBUS_0_RX, 0, 1, 0,
		       q6voice_mixer_get, q6voice_mixer_put),
};

static const struct snd_kcontrol_new q6voice_quat_rx_mixer[] = {
	SOC_SINGLE_EXT("CS-Voice", AFE_PORT_ID_QUATERNARY_MI2S_RX, 0, 1, 0,
		       q6voice_mixer_get, q6voice_mixer_put),
};

static const struct snd_soc_dapm_widget q6voice_widgets[] = {
	SND_SOC_DAPM_AIF_IN("CS-VOICE_DL1", "CS-VOICE Playback", 0,
			    SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("CS-VOICE_UL1", "CS-VOICE Capture", 0,
			     SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_MIXER("CS-Voice Capture Mixer", SND_SOC_NOPM, 0, 0,
			   q6voice_tx_mixer, ARRAY_SIZE(q6voice_tx_mixer)),
	/* The amplifier feedback on QUAT MI2S TX: no codec, so give it a source */
	SND_SOC_DAPM_INPUT("Speaker Feedback"),
	SND_SOC_DAPM_MIXER("CS-Voice EC Mixer", SND_SOC_NOPM, 0, 0,
			   q6voice_ec_mixer, ARRAY_SIZE(q6voice_ec_mixer)),
	SND_SOC_DAPM_MIXER("SLIMBUS_0_RX Voice Mixer", SND_SOC_NOPM, 0, 0,
			   q6voice_slim0_rx_mixer,
			   ARRAY_SIZE(q6voice_slim0_rx_mixer)),
	SND_SOC_DAPM_MIXER("QUAT_MI2S_RX Voice Mixer", SND_SOC_NOPM, 0, 0,
			   q6voice_quat_rx_mixer,
			   ARRAY_SIZE(q6voice_quat_rx_mixer)),
};

static const struct snd_soc_dapm_route q6voice_routes[] = {
	{ "CS-Voice Capture Mixer", "SLIMBUS_0_TX", "SLIMBUS_0_TX" },
	{ "CS-VOICE_UL1", NULL, "CS-Voice Capture Mixer" },
	{ "Quaternary MI2S Capture", NULL, "Speaker Feedback" },
	{ "CS-Voice EC Mixer", "QUAT_MI2S_TX", "QUAT_MI2S_TX" },
	{ "CS-VOICE_UL1", NULL, "CS-Voice EC Mixer" },
	{ "SLIMBUS_0_RX Voice Mixer", "CS-Voice", "CS-VOICE_DL1" },
	{ "SLIMBUS_0_RX", NULL, "SLIMBUS_0_RX Voice Mixer" },
	{ "QUAT_MI2S_RX Voice Mixer", "CS-Voice", "CS-VOICE_DL1" },
	{ "QUAT_MI2S_RX", NULL, "QUAT_MI2S_RX Voice Mixer" },
};

static const struct snd_soc_component_driver q6voice_component = {
	.name = "q6voice-dai",
	.open = q6voice_pcm_open,
	.pcm_new = q6voice_pcm_new,
	.pointer = q6voice_pcm_pointer,
	.trigger = q6voice_pcm_trigger,
	.close = q6voice_pcm_close,
	.read = q6voice_reg_read,
	.write = q6voice_reg_write,
	.dapm_widgets = q6voice_widgets,
	.num_dapm_widgets = ARRAY_SIZE(q6voice_widgets),
	.dapm_routes = q6voice_routes,
	.num_dapm_routes = ARRAY_SIZE(q6voice_routes),
	/* The routes name q6afe back-end widgets */
	.probe_order = SND_SOC_COMP_ORDER_LATE,
};

#define Q6VOICE_SVC_DRIVER(_name, _compat)				\
static int q6##_name##_probe(struct apr_device *adev)			\
{									\
	if (&q6voice_##_name == &q6voice_mvm)				\
		return q6voice_mvm_probe(adev);				\
	return q6voice_probe_svc(adev, &q6voice_##_name);		\
}									\
static void q6##_name##_remove(struct apr_device *adev)		\
{									\
	q6voice_remove_svc(&q6voice_##_name);				\
}									\
static int q6##_name##_callback(struct apr_device *adev,		\
				const struct apr_resp_pkt *data)	\
{									\
	return q6voice_callback_svc(adev, data, &q6voice_##_name);	\
}									\
static const struct of_device_id q6##_name##_device_id[] = {		\
	{ .compatible = _compat },					\
	{}								\
};									\
MODULE_DEVICE_TABLE(of, q6##_name##_device_id);			\
static struct apr_driver q6##_name##_driver = {			\
	.probe = q6##_name##_probe,					\
	.remove = q6##_name##_remove,					\
	.callback = q6##_name##_callback,				\
	.driver = {							\
		.name = "qcom-q6" #_name,				\
		.of_match_table = q6##_name##_device_id,		\
	},								\
}

static int q6voice_mvm_probe(struct apr_device *adev)
{
	int ret;

	ret = q6voice_probe_svc(adev, &q6voice_mvm);
	if (ret)
		return ret;

	return devm_snd_soc_register_component(&adev->dev, &q6voice_component,
					       q6voice_dais,
					       ARRAY_SIZE(q6voice_dais));
}

Q6VOICE_SVC_DRIVER(mvm, "qcom,q6mvm");
Q6VOICE_SVC_DRIVER(cvs, "qcom,q6cvs");
Q6VOICE_SVC_DRIVER(cvp, "qcom,q6cvp");

static struct apr_driver * const q6voice_drivers[] = {
	&q6mvm_driver,
	&q6cvs_driver,
	&q6cvp_driver,
};

static int __init q6voice_init(void)
{
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(q6voice_drivers); i++) {
		ret = apr_driver_register(q6voice_drivers[i]);
		if (ret)
			goto err;
	}

	return 0;

err:
	while (i--)
		apr_driver_unregister(q6voice_drivers[i]);

	return ret;
}
module_init(q6voice_init);

static void __exit q6voice_exit(void)
{
	int i;

	scoped_guard(mutex, &q6voice_lock)
		q6voice_stop_voice();

	for (i = ARRAY_SIZE(q6voice_drivers); i--; )
		apr_driver_unregister(q6voice_drivers[i]);
}
module_exit(q6voice_exit);

MODULE_DESCRIPTION("Q6 Core Voice Driver client");
MODULE_LICENSE("GPL");
