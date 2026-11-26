#include <Mech368Protocol.h>
#include <cassert>
#include <cstring>
#include <iostream>
using namespace mech368::ble;

static Frame command(Receiver& r, uint32_t now, uint16_t seq,
                     Type type = Type::Motion) {
  r.tick(now);
  Frame f = r.status(now); f.type = type; f.sequence = seq;
  f.linear = type == Type::Motion ? 123 : 0;
  f.angular = type == Type::Motion ? -456 : 0;
  return f;
}
static bool deliver(Receiver& r, const Frame& f, uint32_t now) {
  uint8_t bytes[kFrameSize]; assert(encode(f, bytes, sizeof(bytes)));
  return r.receive(bytes, sizeof(bytes), now);
}
static bool status(RemoteLink& link, Frame f, uint32_t now, bool* stopAcknowledged = nullptr) {
  uint8_t bytes[kFrameSize]; assert(encode(f, bytes, sizeof(bytes)));
  return link.status(bytes, sizeof(bytes), now, stopAcknowledged);
}
static void codec() {
  const uint8_t check[] = "123456789";
  assert(crc16(check, 9) == 0x29b1); // independent CRC-16/CCITT-FALSE check vector
  Frame f; f.session = 0x12345678; f.challenge = 0x90abcdef;
  f.sequence = 0x1234; f.linear = -1000; f.angular = 4000;
  uint8_t bytes[kFrameSize]; assert(encode(f, bytes, sizeof(bytes)));
  const uint8_t prefix[] = {0x36,0x8b,1,1,0x34,0x12,0x78,0x56,0x34,0x12,
                           0x18,0xfc,0xa0,0x0f,0xef,0xcd,0xab,0x90};
  assert(std::memcmp(bytes,prefix,sizeof(prefix))==0);
  Frame decoded; assert(decode(bytes,sizeof(bytes),decoded));
  assert(decoded.linear==-1000 && decoded.angular==4000 && decoded.session==f.session);
  for (size_t n=0;n<kFrameSize;++n) assert(!decode(bytes,n,decoded));
  assert(!decode(bytes,kFrameSize+1,decoded)); assert(!decode(nullptr,kFrameSize,decoded));
  for (size_t i=0;i<kFrameSize;++i) for (unsigned b=0;b<8;++b) {
    bytes[i]^=1U<<b; assert(!decode(bytes,sizeof(bytes),decoded)); bytes[i]^=1U<<b;
  }
  // Semantic invalidity with a recomputed, valid checksum.
  const size_t offsets[] = {0,2,3,10,12};
  const uint8_t values[] = {0,2,99,0xff,0xff};
  for (size_t i=0;i<5;++i) {
    uint8_t bad[kFrameSize]; std::memcpy(bad,bytes,sizeof(bad)); bad[offsets[i]]=values[i];
    if (i>=3) bad[offsets[i]+1]=0x7f;
    put16(bad+18,crc16(bad,18)); assert(!decode(bad,sizeof(bad),decoded));
  }
  f.linear=1001; assert(!encode(f,bytes,sizeof(bytes)));
  f.linear=-1001; assert(!encode(f,bytes,sizeof(bytes)));
  f.linear=0; f.angular=-4001; assert(!encode(f,bytes,sizeof(bytes)));
  f.type=Type::Stop; f.angular=1; assert(!encode(f,bytes,sizeof(bytes)));
  f.type=Type::Status; f.angular=0; f.linear=16; assert(!encode(f,bytes,sizeof(bytes)));
}
static void freshnessAndSafety() {
  Receiver r; assert(!r.allowed(0)); r.connect(123,0); assert(!r.allowed(0));
  Frame first=command(r,0,65535); assert(deliver(r,first,1));
  assert(r.allowed(1) && r.linear(1)==123 && r.angular(1)==-456);
  assert(!deliver(r,first,2)); // consumed challenge
  Frame duplicate=command(r,2,65535); assert(!deliver(r,duplicate,2));
  assert(!deliver(r,command(r,2,65534),2));
  assert(!deliver(r,command(r,2,32767),2)); // ambiguous half-range
  assert(deliver(r,command(r,3,0),3)); // sequence rollover
  Frame wrong=command(r,4,1); wrong.session=124; assert(!deliver(r,wrong,4));
  Frame expired=command(r,4,1); assert(!deliver(r,expired,203)); // 200 ms exact
  assert(r.allowed(452)); assert(!r.allowed(453)); // rejects did not refresh
  assert(r.linear(453)==0 && r.angular(453)==0);
  assert(deliver(r,command(r,454,1,Type::Stop),454));
  assert(r.status(454).linear & EstopLatched);
  assert(deliver(r,command(r,455,2),455)); assert(!r.allowed(455));
  r.disconnect(456); r.connect(124,457);
  assert(r.status(457).linear & EstopLatched);
  assert(!deliver(r,wrong,458)); // challenge/sequence from old traffic
  r.tick(459,true); assert(!r.allowed(459));
  assert(deliver(r,command(r,460,0),460)); assert(r.allowed(460));
  r.disconnect(461); assert(!r.allowed(461));
  r.connect(125,462); assert(!r.allowed(462));
  assert(!deliver(r,first,463));
  assert(!deliver(r,r.status(463),463));
  // Clock rollover, challenge validity, watchdog exact boundary.
  Receiver wrap; const uint32_t start=UINT32_MAX-100;
  wrap.connect(22,start); assert(deliver(wrap,command(wrap,start,0),start));
  assert(wrap.allowed(start+449U)); assert(!wrap.allowed(start+450U));
  Frame stale=command(wrap,start,1); assert(!deliver(wrap,stale,start+200U));
  assert(deliver(wrap,command(wrap,start+201U,1),start+201U));
}
static void remoteAck() {
  Receiver r; RemoteLink remote; r.connect(456,0);
  assert(status(remote,r.status(0),0)); assert(remote.ready()); assert(!remote.healthy(0));
  Frame f=remote.command(false); remote.sent(); assert(deliver(r,f,1));
  Frame ack=r.status(1); assert(status(remote,ack,2)); assert(remote.healthy(451));
  assert(status(remote,ack,451)); assert(!remote.healthy(452)); // duplicate ACK
  Frame wrong=ack; wrong.session++; assert(!status(remote,wrong,452));
  wrong=ack; wrong.type=Type::Stop; wrong.linear=0; assert(!status(remote,wrong,452));
  wrong=ack; wrong.sequence++; assert(status(remote,wrong,452)); assert(!remote.healthy(452));
  remote.reset(); assert(!remote.ready() && !remote.healthy(453));
  // Delayed ACK for an older send cannot falsely establish health.
  status(remote,ack,454); remote.sent(); remote.sent();
  assert(status(remote,ack,455)); assert(!remote.healthy(455));
}
static void exchangeAndLoss() {
  Receiver robot; RemoteLink remote;
  robot.connect(77, 0);
  for (uint32_t t = 0; t <= 1000; t += kSendMs) {
    robot.tick(t);
    assert(status(remote, robot.status(t), t));
    Frame tx = remote.command(false);
    remote.sent();
    assert(deliver(robot, tx, t + 1));
    if (t >= kSendMs) assert(remote.healthy(t));
  }
  // Keep delivering cached frames/status through a connected but dead peer.
  Frame replay = remote.command(false);
  Frame frozenStatus = robot.status(1001);
  assert(status(remote, frozenStatus, 1002));
  replay.sequence = frozenStatus.sequence; // duplicate, even with current token
  for (uint32_t t = 1050; t <= 1500; t += kSendMs) {
    assert(!deliver(robot, replay, t));
    assert(status(remote, frozenStatus, t));
  }
  assert(!robot.allowed(1500)); assert(!remote.healthy(1500));
  // A new connection starts inhibited and can establish fresh zero heartbeats.
  robot.disconnect(1501); remote.reset(); robot.connect(78,1502);
  assert(!robot.allowed(1502));
  assert(status(remote,robot.status(1502),1502));
  Frame stop=remote.command(true); remote.sent(true);
  bool stopAcknowledged = true;
  assert(status(remote, robot.status(1502), 1502, &stopAcknowledged) && !stopAcknowledged);
  assert(deliver(robot,stop,1503));
  assert(status(remote,robot.status(1503),1504,&stopAcknowledged) && stopAcknowledged);
  assert(status(remote,robot.status(1503),1505,&stopAcknowledged) && !stopAcknowledged);
  assert(remote.healthy(1504)); assert(!robot.allowed(1504));
  assert(robot.status(1504).linear & EstopLatched);
  // Remote ACK timer remains correct through millis rollover.
  RemoteLink wrap; Receiver peer; uint32_t start=UINT32_MAX-100;
  peer.connect(88,start); status(wrap,peer.status(start),start);
  Frame tx=wrap.command(false); wrap.sent(); assert(deliver(peer,tx,start));
  status(wrap,peer.status(start),start);
  assert(wrap.healthy(start+449U)); assert(!wrap.healthy(start+450U));
}
int main() {
  codec(); freshnessAndSafety(); remoteAck(); exchangeAndLoss();
  std::cout << "BLE codec, freshness, watchdog integration and ACK assertions passed\n";
}
