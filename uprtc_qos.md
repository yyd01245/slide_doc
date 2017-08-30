
# 交互式音视频 QoS 保障技术
<style type="text/css">
  .reveal p {
    text-align: left;
    font-size: 25px;
  }
</style>

影响实时音视频质量的主要因素包括：带宽、延时、抖动、丢包。

* what's qos
* rtp/rtcp
* 抗丢包
* 网络拥塞控制
* 关键帧请求
* PLC


---

## qos 概念

<style type="text/css">
  .reveal p {
    text-align: left;
    font-size: 25px;
  }
  .reveal {
    text-align: left;
    font-size: 25px;
  }  
</style>


![qos_qoe](https://upmedia.b0.upaiyun.com/assets/qos/qos_qoe.png)

QoS（Quality of Service）指一个网络能够利用各种基础技术，为指定的网络通讯提供更好的服务能力, 是网络的一种安全机制， 是用来解决网络延迟和阻塞等问题的一种技术。所以当网络过载或拥塞时，QoS 能确保重要业务量不受延迟或丢弃，同时保证网络的高效运行

常用的QoS保障技术包括IP优先权，码率自适应，丢包重传（ARQ），前向纠错（FEC），后向纠错（PLC）等，这些QoS策略在一定程度上能控制数据包拥塞，消除传输中产生的差错，提高音视频质量。

---

无 qos 机制的流媒体传输方案

![qos_qoe](https://upmedia.b0.upaiyun.com/assets/qos/no_qos.png)

---


## rtp/rtcp 

<style type="text/css">
  .reveal {
    text-align: left;
    font-size: 25px;
  }  
</style>

rtp/rtcp 协议 参考 rfc3550

交互式实时视频应用通常采用 RTP 协议进行音视频传输，RTP头部提供了诸如负载类型、时间戳、序列号和同步源等信息保证基本的音视频实时传输需求。RTP协议底层采用不可靠的 UDP 传输层协议，当网络过载或拥塞，无法实现对丢包、抖动、乱序以及网络拥塞的自适应调整。

RTP 是一个应用型的传输层协议，位于 UDP 上，本身只保证实时数据的传输，不提供任何传输可靠性的保证和流量的拥塞控制机制，RTCP 协议则负责流媒体的传输质量保证，提供流量控制和拥塞控制等机制。在 RTP 会话期间，各参与者周期性彼此发送 RTCP 报文，报文中包含各参与者数据发送和接收等统计信息，参与者可以根据报文中信息动态控制流媒体传输。在 WebRTC 项目中，RTP/RTCP 作为传输模块的一部分，负责对发送端采集到的媒体数据进行进行封包，然后交给上层网络模块发送；在接收端 RTP/RTCP 模块收到上层模块的数据包后，进行解包操作，最后把负载发送到解码模块。因此可以说 RTP/RTCP 是 WebRTC 的重要基础。RFC3550／3551 定义 RTP/RTCP 协议的基本内容，包括报文格式、传输规则等。

![qos_qoe](https://upmedia.b0.upaiyun.com/assets/qos/rtp_header.png)


RTCP 周期性发送控制包信息。

* SR：发送者报告，描述作为活跃发送者成员的发送和接收统计数字；
* RR：接收者报告，描述非活跃发送者成员的接收统计数字；

---




## 抗丢包

<style type="text/css">
  .reveal {
    text-align: left;
    font-size: 25px;
  }  
</style>

抗丢包策略主要是两种思路:

* 丢包重传(ARQ)
* 前向纠错(FEC)

---

### 丢包重传(ARQ)

否定应答(NACK)

发送者通过接受者的反馈得知有报文在传输过程中有丢失，重传该报文。

基于NACK反馈的丢包重传方法(RFC4585)：接收端循环检查接收缓冲，当发现丢包后使用RTCPNACK反馈报文将丢包信息反馈给发送端；发送端接收NACK反馈并解析后从发送缓存取出对应RTP包，并再次发送给接收端。该方法的缺点是增大了端到端的延迟，尤其在丢包大量发生时更为明显。

---

### 前向纠错(FEC)
在编码发送端发送冗余数据，解码端根据该冗余数据，对丢失的码流进行恢复


前向纠错FEC：FEC机制是在接收端根据视频帧的重要性（参考帧或非参考帧）发送冗余的视频RTP包，在接收端如果检测到丢包则利用冗余包进行恢复，否则将冗余包丢弃。该方法的优点是视频无延迟，但发送冗余包占用了额外的带宽资源。


比较理想的抗丢包方案 nack + fec 模式

接收端根据帧大小和接收时延估计可用带宽，发送端根据可用带宽、丢包和RTT等反馈计算分配保护开销（protectionoverhead，包括FEC bitrate、NACK bitrate）和视频编码码率各占的比率。具体来说，FEC的保护级别（protectionlevel）取决于往返时间RTT，当RTT较小时，丢包重传的延时不会导致明显的视频卡顿，因此可以相应减少FEC包的数量；当RTT较大时，时延对视频流畅度影响明显，因此要相应增加FEC包的数量。此外，可以使用多帧FEC和结合时域分层信息的FEC，二者都可以在减小保护开销的同时，提供更低的渲染抖动、更低的端到端延迟和更高的视频质量。

---



## 网络拥塞控制(gcc)

<style type="text/css">
  .reveal {
    text-align: left;
    font-size: 25px;
  }  
</style>


如何实现码率自适应, 接收端估算最大接收码率, 通过 rtcp 会定时发送 RMEB(Receiver Estimated Maximum Bitrate) 数据, 如何估算最大接收码率, 采用两种方式进行估算:

* 基于丢包; 
* 基于延时;

---

### 基于丢包

丢包率与带宽有时没有直接关系，导致误判。即丢包很多时候并非由于网络带宽不够用；

历史估计未来，缺乏预判性。假定丢包是因为带宽不够用导致，等感知到丢包，说明网络已经拥塞，这将导致视频卡顿等问题；另一方面，通过感知到丢包到应用层才去策略，需要一定的时间，这段时间将进一步导致拥塞。这在对延迟要求高的场景下，效果不理想。

![qos_qoe](https://upmedia.b0.upaiyun.com/assets/qos/loss_estimate.png)

---

## 基于延时

接收端采用 Google Congestion Control 算法，简称GCC。
具体算法详情:
https://tools.ietf.org/html/draft-alvestrand-rtcweb-congestion-02

![qos_qoe](https://upmedia.b0.upaiyun.com/assets/qos/gcc_process.png)

实时统计当前接收码率, 计算帧间延时差 d(i)，计算帧间数据长度差值 dL(i).

![qos_qoe](https://upmedia.b0.upaiyun.com/assets/qos/gcc_ts_delta.png)

T(i) 是第 i 帧中第一个数据包的发送时间，t(i) 是第 i 帧中最后一个数据包的到达时间。帧间延迟通过如下公式计算得到

d(i) = t(i) – t(i-1) – (T(i) – T(i-1))

dL(i) = L(i) - L(i-1)

带宽估算模型： d(i) = dL(i) / C(i) + m(i) + v(i)    

dL(i)表示帧i和帧i-1的长度之差，C(i)表示信道传输速率，m(i)表示帧i的网络排队时延，v(i)表示测量噪声，其协方差为R。其中[1/C(i) m(i)]是我们要求的目标值，即信道传输速率和网络排队时延。

d(i)两帧数据的网络传输时间差，dL(i)两帧数据的大小差， c为网络传输能力， w(i)是我们关注的重点， 它主要由三个因素决定：发送速率， 网络路由能力， 以及网络传输能力。w(i)符合高斯分布， 有如下结论：当w(i)增加时，占用过多带宽（over-using)；当w(i)减少时，占用较少带宽（under-using)；为0时，用到恰好的带宽。所以，只要我们能计算出w(i)，即能判断目前的网络使用情况，从而增加或减少发送的速率。

构建卡夫曼滤波模型, 估算出 m(i), 网络状态检测用当前网络延迟 m(i) 和阈值(动态更新) 进行比较，判断出overuse，underuse 和 normal 三种网络状态之一. 码率控制模块维护三种码率变化趋势(Hold, Increase, Decrease),接下来根据估算真正的码率 Ar. 

```
   State ---->  | Hold      |Increase    |Decrease
   Signal-----------------------------------------
     v          |           |            |
   Over-use     | Decrease  |Decrease    |
   -----------------------------------------------
   Normal       | Increase  |            |Hold
   -----------------------------------------------
   Under-use    |           |Hold        |Hold
   -----------------------------------------------

```

当码率变化趋势为Increase时，当前码率为上次码率乘上系数1.05；当码率变化趋势为Decrease，当前码率为过去500ms内的最大接收码率乘上系数0.85。当码率变化趋势为Hold时，当前码率保持不变。

----


```
void OveruseEstimator::Update(int64_t t_delta,
                              double ts_delta,
                              int size_delta,
                              BandwidthUsage current_hypothesis,
                              int64_t now_ms) {
  const double min_frame_period = UpdateMinFramePeriod(ts_delta);
  const double t_ts_delta = t_delta - ts_delta;
  BWE_TEST_LOGGING_PLOT(1, "dm_ms", now_ms, t_ts_delta);
  double fs_delta = size_delta;

  ++num_of_deltas_;
  if (num_of_deltas_ > kDeltaCounterMax) {
    num_of_deltas_ = kDeltaCounterMax;
  }

  // Update the Kalman filter.
  E_[0][0] += process_noise_[0];
  E_[1][1] += process_noise_[1];

  if ((current_hypothesis == BandwidthUsage::kBwOverusing &&
       offset_ < prev_offset_) ||
      (current_hypothesis == BandwidthUsage::kBwUnderusing &&
       offset_ > prev_offset_)) {
    E_[1][1] += 10 * process_noise_[1];
  }

  const double h[2] = {fs_delta, 1.0};
  const double Eh[2] = {E_[0][0]*h[0] + E_[0][1]*h[1],
                        E_[1][0]*h[0] + E_[1][1]*h[1]};

  BWE_TEST_LOGGING_PLOT(1, "d_ms", now_ms, slope_ * h[0] - offset_);

  const double residual = t_ts_delta - slope_*h[0] - offset_;

  const bool in_stable_state =
      (current_hypothesis == BandwidthUsage::kBwNormal);
  const double max_residual = 3.0 * sqrt(var_noise_);
  // We try to filter out very late frames. For instance periodic key
  // frames doesn't fit the Gaussian model well.
  if (fabs(residual) < max_residual) {
    UpdateNoiseEstimate(residual, min_frame_period, in_stable_state);
  } else {
    UpdateNoiseEstimate(residual < 0 ? -max_residual : max_residual,
                        min_frame_period, in_stable_state);
  }

  const double denom = var_noise_ + h[0]*Eh[0] + h[1]*Eh[1];

  const double K[2] = {Eh[0] / denom,
                       Eh[1] / denom};

  const double IKh[2][2] = {{1.0 - K[0]*h[0], -K[0]*h[1]},
                            {-K[1]*h[0], 1.0 - K[1]*h[1]}};
  const double e00 = E_[0][0];
  const double e01 = E_[0][1];

  // Update state.
  E_[0][0] = e00 * IKh[0][0] + E_[1][0] * IKh[0][1];
  E_[0][1] = e01 * IKh[0][0] + E_[1][1] * IKh[0][1];
  E_[1][0] = e00 * IKh[1][0] + E_[1][0] * IKh[1][1];
  E_[1][1] = e01 * IKh[1][0] + E_[1][1] * IKh[1][1];

  // The covariance matrix must be positive semi-definite.
  bool positive_semi_definite = E_[0][0] + E_[1][1] >= 0 &&
      E_[0][0] * E_[1][1] - E_[0][1] * E_[1][0] >= 0 && E_[0][0] >= 0;
  assert(positive_semi_definite);
  if (!positive_semi_definite) {
    LOG(LS_ERROR) << "The over-use estimator's covariance matrix is no longer "
                     "semi-definite.";
  }

  slope_ = slope_ + K[0] * residual;
  prev_offset_ = offset_;
  offset_ = offset_ + K[1] * residual;

  BWE_TEST_LOGGING_PLOT(1, "kc", now_ms, K[0]);
  BWE_TEST_LOGGING_PLOT(1, "km", now_ms, K[1]);
  BWE_TEST_LOGGING_PLOT(1, "slope_1/bps", now_ms, slope_);
  BWE_TEST_LOGGING_PLOT(1, "var_noise", now_ms, var_noise_);
}

```





---

统计码率状态

![qos_qoe](https://upmedia.b0.upaiyun.com/assets/qos/gcc_estimate.png)

---

## 关键帧请求

<style type="text/css">
  .reveal {
    text-align: left;
    font-size: 25px;
  }  
</style>

关键帧也叫做即时刷新帧，简称IDR帧。对视频来说，IDR帧的解码无需参考之前的帧，因此在丢包严重时可以通过发送关键帧请求进行画面的恢复。关键帧的请求方式分为三种：RTCP FIR反馈（Full intra frame request）、RTCP PLI反馈（Picture Loss Indictor），具体使用哪种可通过协商确定, uprtc 直接使用了 PLI 和 FIR 两种模式。uprtc 通过关键帧请求实现秒开的功能。

![qos_qoe](https://upmedia.b0.upaiyun.com/assets/qos/I_frame.png)


---

## 抖动、乱序

<style type="text/css">
  .reveal {
    text-align: left;
    font-size: 25px;
  }  
</style>

抖动, 乱序主要依赖于 jitter buffer 修正, 但同时也会与其他的 qos 策略紧密相连。当有些包很久（一般大于500ms）都没收到，就认为它来不了，直接将它们丢弃；有些包短时间（小于50ms）没来，则向发送端发送重传请求，请求发送端再发一次该包，试图能补上这些包(nack). 丢帧则会请求 fir. 调整 jitter buffer 将牺牲延时为代价, 需要在延时和丢包、抖动之间做出平衡。

---

## PLC（Packet Loss Concealment）丢包隐藏算法暂时仅应用在音频, IP 网络中出现音频数据包丢失时会导致语音失真，为了减轻信包丢失对语音感知质量的影响，因为在声音信号特别是语音信 号中存在大量的短期自相似性。PLC 算法利用丢失信包的前一信包或邻接信包（在后一信包可获得的情况下）预测丢失的数据包，尽可能地恢复出原来的语音信息。PLC 算法多数基于接收端处理，不需要发送端参与。属于后向纠错。丢包隐藏技术这类技术适用于相对小的丢包率( 有研究者说是小于 15%)和 小 长 度 的 包(4~40ms).```
int WebRtcOpus_Decode(OpusDecInst* inst, const uint8_t* encoded,
                      size_t encoded_bytes, int16_t* decoded,
                      int16_t* audio_type) {
  int decoded_samples;

  if (encoded_bytes == 0) {
    *audio_type = DetermineAudioType(inst, encoded_bytes);
    decoded_samples = WebRtcOpus_DecodePlc(inst, decoded, 1);
  } else {
    decoded_samples = DecodeNative(inst,
                                   encoded,
                                   encoded_bytes,
                                   kWebRtcOpusMaxFrameSizePerChannel,
                                   decoded,
                                   audio_type,
                                   0);
  }
  if (decoded_samples < 0) {
    return -1;
  }

  /* Update decoded sample memory, to be used by the PLC in case of losses. */
  inst->prev_decoded_samples = decoded_samples;

  return decoded_samples;
}

```---


## IP优先级

IP优先级利用IP报文中的优先权部分，对音频、视频和RTCP数据流进行优先级划分，IP报文的包头中有一专用字节，称之为服务类型域“该字节前三个比特位用来定义数据报优先等级”IP优先是描述0-7等8个不同的优先等级。音视频会议系统中，当网络带宽低于一定标准时，可及时调整包的优先级级别，这样可帮助路由器选择IP包的发送与接收的优先级。一般来说音频包对时间延迟最为敏感，当网络采用IP优先权进行流量匹配时，可通过视频设备发出的修改过IP优先权字段信息的视音频包进行入队列处理，以保证音视频会议码流的优先传送。

---

## 结语

QOS保障技术，不同的技术对于不同的网络状况效果不同，比如ARQ重传在小丢包和较低网络延迟的网络情况下能够达到较理想的效果, 音视频QOS的关键问题是如何能够整合这些技术，使得各个技术能够发挥到最佳效果，从而提高音视频的体验。

在互联网上为实时交互式音视频应用提供 QoS 保证仍是一项挑战，需要音视频编码器、传输、预处理等多模块的协作配合，或利用现有网络协议和设备的支持，才能提供给客户更多的选择和服务保证。

