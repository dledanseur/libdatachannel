/**
 * Copyright (c) 2020 Staz Modrzynski
 * Copyright (c) 2020 Paul-Louis Ageneau
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#ifndef RTC_RTCP_RECEIVING_SESSION_H
#define RTC_RTCP_RECEIVING_SESSION_H

#if RTC_ENABLE_MEDIA

#include "common.hpp"
#include "mediahandler.hpp"
#include "message.hpp"
#include "rtp.hpp"

#define RTP_SEQ_MOD (1<<16)

namespace rtc {

// An RtcpSession can be plugged into a Track to handle the whole RTCP session
class RTC_CPP_EXPORT RtcpReceivingSession : public MediaHandler {
public:
	message_ptr incoming(message_ptr ptr) override;
	message_ptr outgoing(message_ptr ptr) override;
	bool send(message_ptr ptr);

	void requestBitrate(unsigned int newBitrate);

	bool requestKeyframe() override;

protected:
	void pushREMB(unsigned int bitrate);
	void pushRR(unsigned int lastSR_delay);

	void pushPLI();

	void initSeq(uint16_t seq);
	bool updateSeq(uint16_t seq);

	unsigned int mRequestedBitrate = 0;
	SSRC mSsrc = 0;
	uint32_t mGreatestSeqNo = 0;


    uint16_t mMaxSeq = 0;			// highest seq. number seen
    uint32_t mCycles = 0;			// shifted count of seq. number cycles
	uint32_t mBaseSeq = 0;			// base seq number
    uint32_t mBadSeq = 0;			// last 'bad' seq number + 1
	uint32_t mProbation = 0;		// sequ. packets till source is valid
    uint32_t mReceived = 0;			// packets received
    uint32_t mExpectedPrior = 0;	// packet expected at last interval
    uint32_t mReceivedPrior = 0;	// packet received at last interval
	uint32_t mTransit = 0;			// relative trans time for prev pkt
	uint32_t mJitter = 0;			// estimated jitter

	uint64_t mSyncRTPTS, mSyncNTPTS;
};

} // namespace rtc

#endif // RTC_ENABLE_MEDIA

#endif // RTC_RTCP_RECEIVING_SESSION_H
