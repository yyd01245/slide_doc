# 交互式音视频 QoS 保障技术

* Qos
* RTP/RTCP
* 抗丢包
* 网络拥塞控制
* 关键帧请求
* Jitter buffer


---

## Qos 概念


![qos_qoe](https://upmedia.b0.upaiyun.com/assets/qos/qos_qoe.png)


影响实时音视频质量的主要因素包括：带宽、延时、抖动、丢包。Qos（Quality of Service）就是为处理这些问题而生.


---

<style type="text/css">
  .reveal {
    text-align: left;
    font-size: 18px;
  }  
</style>

传统流媒体传输方案

![qos_qoe](https://upmedia.b0.upaiyun.com/assets/qos/no_qos.png)


---

Qos 保障传输方案

![qos_qoe](https://upmedia.b0.upaiyun.com/assets/qos/feed_back.png)

---

## RTP


交互式实时视频应用通常采用 RTP 协议进行音视频传输，RTP头部提供了诸如负载类型、时间戳、序列号和同步源等信息保证基本的音视频实时传输需求。(RFC3550)

![qos_qoe](https://upmedia.b0.upaiyun.com/assets/qos/rtp_header.png)

---

## Rtcp 


* SR：发送者报告，描述作为活跃发送者成员的发送和接收统计数字.
* RR：接收者报告，描述非活跃发送者成员的接收统计数字.

---


## 抗丢包


* 丢包重传(ARQ)
* 前向纠错(FEC)
* 丢包隐藏(PLC)

---

### 丢包重传(ARQ)


否定应答(NACK): 发送者通过接受者的反馈得知有报文在传输过程中有丢失，重传该报文。(RFC4585)

该方法的缺点是增大了端到端的延迟，尤其在丢包大量发生时更为明显。

---

### 前向纠错(FEC)


![qos_qoe](https://upmedia.b0.upaiyun.com/assets/qos/fec.jpg)在编码发送端发送冗余数据，解码端根据该冗余数据，对丢失的码流进行恢复.(RFC6363)

该方法的优点是视频延迟低，但发送冗余包占用了额外的带宽资源。

---

### 丢包隐藏(PLC)Packet Loss Concealment, 属于后向纠错, PLC 算法多数基于接收端处理, 不需要发送端参与。PLC 算法利用丢失信包的前一信包或邻接信包（在后一信包可获得的情况下）预测丢失的数据包. 丢包隐藏技术这类技术适用于相对小的丢包率( 有研究者说是小于 15%)和 小 长 度 的 包(4~40ms).---#### Opus PLC 模块```
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



## 网络拥塞控制

### 基于丢包

* 丢包率与带宽有时没有直接关系，导致误判。即丢包很多时候并非由于网络带宽不够用；
* 历史估计未来，缺乏预判性。假定丢包是因为带宽不够用导致，等感知到丢包，说明网络已经拥塞，这将导致视频卡顿等问题；
* 通过感知到丢包到应用层才去策略，需要一定的时间，这段时间将进一步导致拥塞。这在对延迟要求高的场景下，效果不理想。

---

![qos_qoe](https://upmedia.b0.upaiyun.com/assets/qos/loss_estimate.png)

---

### 基于延时

接收端采用 Google Congestion Control 算法，简称GCC。
[GCC](https://tools.ietf.org/html/draft-alvestrand-rtcweb-congestion-02)


![qos_qoe](https://upmedia.b0.upaiyun.com/assets/qos/gcc_estimate.png)

---


GCC 算法处理流程

![qos_qoe](https://upmedia.b0.upaiyun.com/assets/qos/gcc_process.png)



---



![qos_qoe](https://upmedia.b0.upaiyun.com/assets/qos/gcc_ts_delta.png)

* d(i) = t(i) – t(i-1) – (T(i) – T(i-1))
* dL(i) = L(i) - L(i-1)
* 延时估算模型： d(i) = dL(i) / C(i) + m(i) + v(i)    

---


* dL(i) 表示帧 i 和帧 i-1 的长度之差，
* C(i) 表示信道传输速率，
* m(i) 表示帧 i 的网络排队时延，
* v(i) 表示测量噪声，其协方差为 R。
其中 [1/C(i) m(i)] 是我们要求的目标值，即信道传输速率和网络排队时延。

---

* 构建卡尔曼滤波模型, 估算出 m(i)
* 网络状态检测用当前网络延迟 m(i) 和阈值(动态更新) 进行比较，判断出overuse，underuse 和 normal 三种网络状态之一. 
* 码率控制模块维护三种码率变化趋势(Hold, Increase, Decrease),估算真正的码率 Ar. 
* 当码率变化趋势为 Increase 时，当前码率为上次码率乘上系数 1.05；当码率变化趋势为 Decrease，当前码率为过去 500ms 内的最大接收码率乘上系数 0.85。当码率变化趋势为 Hold 时，当前码率保持不变。

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

---

## 关键帧请求


关键帧也叫做帧内刷新帧，简称 IDR 帧。请求方式分为：

* RTCP FIR反馈（Full intra frame request)
* RTCP PLI反馈（Picture Loss Indictor）

UPRTC 通过关键帧请求实现秒开的功能。

![qos_qoe](https://upmedia.b0.upaiyun.com/assets/qos/I_frame.png)


---

## 抖动、乱序


jitter buffer 主要用于修正抖动, 乱序, 但同时也会与其他的 qos 策略紧密相连。当有丢包则可请求 nack, 丢帧则请求 Fir.

调整 jitter buffer 将牺牲延时为代价, 需要在延时和丢包、抖动之间做出平衡。

![qos_qoe](https://upmedia.b0.upaiyun.com/assets/qos/jitter_buffer.jpg)

---



## 结语


QOS保障技术，不同的技术对于不同的网络状况效果不同，比如 ARQ 重传在小丢包和较低网络延迟的网络情况下能够达到较理想的效果, 音视频 QOS 的关键问题是如何能够整合这些技术，使得各个技术能够发挥到最佳效果，从而提高音视频的体验。

在互联网上为实时交互式音视频应用提供 QoS 保证仍是一项挑战，需要音视频编码器、传输、预处理等多模块的协作配合，或利用现有网络协议和设备的支持，才能提供给客户更多的选择和服务保证。

---

## 谢谢聆听!
