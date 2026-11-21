// 模拟的 ArduinoBLE 协议栈，以及模拟开发板之间的无线信道。
#include "Runtime.h"
#include <algorithm>
#include <cstdio>

namespace fwsim {

Air& air() {
  static Air a;
  return a;
}

std::string boardAddress(int id) {
  char buf[32];
  std::snprintf(buf, sizeof buf, "aa:bb:cc:00:00:%02x", (id + 1) & 0xff);
  return buf;
}

void Air::reset() { *this = Air(); }

Board* Air::board(int id) const {
  for (Board* b : boards)
    if (b->id == id) return b;
  return nullptr;
}

Connection* Air::connection(int id) {
  for (auto& c : connections)
    if (c.id == id) return &c;
  return nullptr;
}

Connection* Air::activeFor(int boardId, bool asCentral) {
  for (auto& c : connections) {
    if (asCentral && c.central == boardId && c.centralUp) return &c;
    if (!asCentral && c.peripheral == boardId && c.peripheralUp) return &c;
  }
  return nullptr;
}

bool Air::reachable(const Board& a, const Board& b) const {
  return config.linkAvailable && a.started && b.started && !a.poweredOff &&
         !b.poweredOff && a.ble.begun && b.ble.begun;
}

uint64_t Air::blocking(uint64_t baseUs) {
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  const double f = 1.0 + config.blockingSpread * u(rng);
  return static_cast<uint64_t>(static_cast<double>(baseUs) * std::max(0.0, f));
}

void Air::post(Board& to, const BleEvent& e) {
  auto& q = to.ble.inbox;
  auto it = q.end();
  while (it != q.begin() && (it - 1)->at > e.at) --it;
  q.insert(it, e);
}

void Air::dropLink(Connection& c, uint64_t now, const std::string& why) {
  if (!c.physical) return;
  c.physical = false;
  c.lostAt = now;
  note(now, "无线：连接 " + std::to_string(c.id) + " 丢失（" + why +
                "）；双方要到监督超时后才会察觉");
  BleEvent e{BleEvent::Disconnected, now + config.supervisionTimeoutUs, c.id, "", {}};
  Board* central = board(c.central);
  Board* peripheral = board(c.peripheral);
  if (central && !central->poweredOff && c.centralUp) post(*central, e);
  if (peripheral && !peripheral->poweredOff && c.peripheralUp) post(*peripheral, e);
}

void Air::closeByLocal(Connection& c, Board& self, uint64_t now) {
  const bool selfIsCentral = c.central == self.id;
  bool& mine = selfIsCentral ? c.centralUp : c.peripheralUp;
  if (!mine) return;
  mine = false;
  BleEvent own{BleEvent::Disconnected, now, c.id, "", {}};
  own.local = true;
  post(self, own);
  Board* peer = board(selfIsCentral ? c.peripheral : c.central);
  if (c.physical && peer && !peer->poweredOff) {
    BleEvent e{BleEvent::Disconnected, now + config.latencyUs, c.id, "", {}};
    post(*peer, e);
  }
  if (c.physical) {
    c.physical = false;
    c.lostAt = now;
  }
  note(now, "无线：" + self.name + " 关闭了连接 " + std::to_string(c.id));
}

void Air::setLinkAvailable(bool available, uint64_t now) {
  if (config.linkAvailable == available) return;
  config.linkAvailable = available;
  note(now, available ? "无线：链路恢复可用" : "无线：链路不可用（超出范围）");
  if (!available)
    for (auto& c : connections) dropLink(c, now, "超出范围");
}

void Air::boardPoweredOff(Board& b, uint64_t now) {
  b.ble.advertising = false;
  b.ble.scanning = false;
  for (auto& c : connections) {
    if (c.central != b.id && c.peripheral != b.id) continue;
    if (c.central == b.id) c.centralUp = false;
    else c.peripheralUp = false;
    dropLink(c, now, b.name + " 断电");
  }
}

void Air::transmit(Connection& c, bool fromCentral, const std::string& uuid,
                   const std::vector<uint8_t>& bytes, BleEvent::Kind kind, uint64_t now) {
  RadioPacket p{now, 0, fromCentral, c.central, c.peripheral, uuid, bytes,
                RadioPacket::Delivered};
  std::uniform_real_distribution<double> u(0.0, 1.0);
  const double loss = fromCentral ? config.lossCentralToPeripheral : config.lossPeripheralToCentral;
  const double corrupt = fromCentral ? config.corruptCentralToPeripheral
                                     : config.corruptPeripheralToCentral;
  const bool blocked = fromCentral ? config.blockCentralToPeripheral
                                   : config.blockPeripheralToCentral;
  Board* to = board(fromCentral ? c.peripheral : c.central);
  if (!c.physical || !to || to->poweredOff) {
    p.fate = RadioPacket::LinkDown;
  } else if (blocked) {
    p.fate = RadioPacket::Blocked;
  } else if (loss > 0 && u(rng) < loss) {
    p.fate = RadioPacket::Lost;
  } else {
    uint64_t at = now + config.latencyUs;
    if (config.jitterUs) at += static_cast<uint64_t>(u(rng) * config.jitterUs);
    uint64_t& last = fromCentral ? c.lastDeliverToPeripheral : c.lastDeliverToCentral;
    at = std::max(at, last);  // BLE 链路按顺序送达
    last = at;
    p.deliverUs = at;
    BleEvent e{kind, at, c.id, uuid, bytes};
    if (corrupt > 0 && !bytes.empty() && u(rng) < corrupt) {
      const size_t bit = static_cast<size_t>(u(rng) * bytes.size() * 8) % (bytes.size() * 8);
      e.bytes[bit / 8] ^= static_cast<uint8_t>(1U << (bit % 8));
      p.fate = RadioPacket::Corrupted;
    }
    post(*to, e);
  }
  if (sniff) sniff(p);
}

static std::shared_ptr<CharState> findLocalChar(Board& b, const std::string& uuid) {
  for (auto& s : b.ble.services)
    for (auto& c : s->chars)
      if (c->uuid == uuid) return c;
  return nullptr;
}

static BLEDevice deviceFor(int viewer, int peer, int conn) {
  auto d = std::make_shared<DeviceState>();
  d->viewer = viewer;
  d->peer = peer;
  d->conn = conn;
  return BLEDevice(d);
}

void Air::process(Board& b) {
  CurrentBoard guard(&b);
  while (!b.ble.inbox.empty() && b.ble.inbox.front().at <= b.clockUs) {
    BleEvent e = b.ble.inbox.front();
    b.ble.inbox.pop_front();
    Connection* c = connection(e.conn);
    if (!c) continue;
    const bool isCentral = c->central == b.id;
    const int peer = isCentral ? c->peripheral : c->central;
    bool& side = isCentral ? c->centralUp : c->peripheralUp;
    switch (e.kind) {
      case BleEvent::Connected:
        if (side && b.ble.handlers[BLEConnected])
          b.ble.handlers[BLEConnected](deviceFor(b.id, peer, c->id));
        break;
      case BleEvent::Disconnected:
        if (!e.local && !side) break;
        side = false;
        if (!isCentral) c->subscribedUuids.clear();
        if (b.ble.handlers[BLEDisconnected])
          b.ble.handlers[BLEDisconnected](deviceFor(b.id, peer, c->id));
        if (!isCentral && b.ble.advertiseWanted && b.ble.connectable &&
            !activeFor(b.id, false)) {
          b.ble.advertising = true;  // 断开后 ArduinoBLE 会重新广播
          b.ble.advertisingSince = b.clockUs;
        }
        break;
      case BleEvent::Write: {
        if (!side || (!c->physical && e.at > c->lostAt)) break;
        auto ch = findLocalChar(b, e.uuid);
        if (!ch) break;
        const size_t n = std::min(e.bytes.size(), static_cast<size_t>(std::max(0, ch->capacity)));
        ch->value.assign(e.bytes.begin(), e.bytes.begin() + static_cast<long>(n));
        ch->writtenFlag = true;
        if (ch->handlers[BLEWritten])
          ch->handlers[BLEWritten](deviceFor(b.id, peer, c->id), BLECharacteristic(ch));
        break;
      }
      case BleEvent::Notify:
        if (!side || (!c->physical && e.at > c->lostAt)) break;
        for (auto& weak : remoteChars) {
          auto ch = weak.lock();
          if (!ch || ch->owner != b.id || ch->conn != c->id || ch->uuid != e.uuid ||
              !ch->remoteSubscribed)
            continue;
          ch->value = e.bytes;
          ch->updatedFlag = true;
          if (ch->handlers[BLEUpdated])
            ch->handlers[BLEUpdated](deviceFor(b.id, peer, c->id), BLECharacteristic(ch));
        }
        break;
      case BleEvent::Subscribed: {
        auto ch = findLocalChar(b, e.uuid);
        if (side && ch && ch->handlers[BLESubscribed])
          ch->handlers[BLESubscribed](deviceFor(b.id, peer, c->id), BLECharacteristic(ch));
        break;
      }
    }
  }
}

}  // namespace fwsim

using namespace fwsim;

// --------------------------------------------------------- BLELocalDevice ---
BLELocalDevice BLE;

int BLELocalDevice::begin() {
  Board& b = cur();
  b.spend(50000);
  b.ble.begun = true;
  return 1;
}
void BLELocalDevice::end() {
  Board& b = cur();
  for (auto& c : air().connections)
    if (c.central == b.id || c.peripheral == b.id) air().closeByLocal(c, b, b.now());
  b.ble.begun = false;
  b.ble.advertising = b.ble.scanning = false;
}
void BLELocalDevice::poll(unsigned long timeout) {
  Board& b = cur();
  b.spend(b.ioCallUs * 10);
  air().process(b);
  if (timeout == 0) return;
  const uint64_t until = b.clockUs + timeout * 1000ULL;
  while (b.clockUs < until && (b.ble.inbox.empty() || b.ble.inbox.front().at > b.clockUs)) {
    const uint64_t next = b.ble.inbox.empty() ? until : std::min(until, b.ble.inbox.front().at);
    b.spend(std::max<uint64_t>(1, next - b.clockUs));
  }
  air().process(b);
}
bool BLELocalDevice::connected() const {
  Board& b = cur();
  return air().activeFor(b.id, false) || air().activeFor(b.id, true);
}
bool BLELocalDevice::disconnect() {
  Board& b = cur();
  bool any = false;
  for (auto& c : air().connections)
    if ((c.central == b.id && c.centralUp) || (c.peripheral == b.id && c.peripheralUp)) {
      air().closeByLocal(c, b, b.now());
      any = true;
    }
  return any;
}
String BLELocalDevice::address() const { return String(boardAddress(cur().id)); }
int BLELocalDevice::rssi() { return connected() ? -50 : 127; }
bool BLELocalDevice::setAdvertisedServiceUuid(const char* uuid) {
  cur().ble.advertisedUuid = lower(uuid);
  return true;
}
bool BLELocalDevice::setAdvertisedService(const BLEService& service) {
  if (!service) return false;
  cur().ble.advertisedUuid = service.state()->uuid;
  return true;
}
bool BLELocalDevice::setLocalName(const char* name) {
  cur().ble.localName = name ? name : "";
  return true;
}
void BLELocalDevice::setDeviceName(const char* name) { cur().ble.deviceName = name ? name : ""; }
void BLELocalDevice::addService(BLEService& service) {
  Board& b = cur();
  if (!service) return;
  for (auto& c : service.state()->chars) c->owner = b.id;
  b.ble.services.push_back(service.state());
}
int BLELocalDevice::advertise() {
  Board& b = cur();
  if (!b.ble.begun) return 0;
  b.ble.advertiseWanted = true;
  if (!air().activeFor(b.id, false) && !b.ble.advertising) {
    b.ble.advertising = true;
    b.ble.advertisingSince = b.now();
    air().note(b.now(), b.name + "：开始广播");
  }
  return 1;
}
void BLELocalDevice::stopAdvertise() {
  Board& b = cur();
  b.ble.advertiseWanted = false;
  b.ble.advertising = false;
}
static int startScan(const std::string& uuid, const std::string& name, const std::string& addr) {
  Board& b = cur();
  if (!b.ble.begun) return 0;
  b.spend(b.ioCallUs * 10);
  b.ble.scanning = true;
  b.ble.scanUuid = lower(uuid);
  b.ble.scanName = name;
  b.ble.scanAddress = lower(addr);
  b.ble.scanSince = b.now();
  b.ble.reported.clear();
  return 1;
}
int BLELocalDevice::scan(bool) { return startScan("", "", ""); }
int BLELocalDevice::scanForName(String name, bool) { return startScan("", name.str(), ""); }
int BLELocalDevice::scanForUuid(String uuid, bool) { return startScan(uuid.str(), "", ""); }
int BLELocalDevice::scanForAddress(String address, bool) { return startScan("", "", address.str()); }
void BLELocalDevice::stopScan() { cur().ble.scanning = false; }
BLEDevice BLELocalDevice::central() {
  Board& b = cur();
  Connection* c = air().activeFor(b.id, false);
  return c ? deviceFor(b.id, c->central, c->id) : BLEDevice();
}
BLEDevice BLELocalDevice::available() {
  Board& b = cur();
  b.spend(b.ioCallUs * 10);
  if (!b.ble.scanning) return BLEDevice();
  for (Board* o : air().boards) {
    if (o == &b || !o->ble.advertising || !air().reachable(b, *o)) continue;
    if (std::find(b.ble.reported.begin(), b.ble.reported.end(), o->id) != b.ble.reported.end())
      continue;
    if (!b.ble.scanUuid.empty() && b.ble.scanUuid != o->ble.advertisedUuid) continue;
    if (!b.ble.scanName.empty() && b.ble.scanName != o->ble.localName) continue;
    if (!b.ble.scanAddress.empty() && b.ble.scanAddress != boardAddress(o->id)) continue;
    const uint64_t since = std::max(b.ble.scanSince, o->ble.advertisingSince);
    if (b.now() < since + air().config.scanFindUs) continue;
    b.ble.reported.push_back(o->id);
    return deviceFor(b.id, o->id, -1);
  }
  return BLEDevice();
}
void BLELocalDevice::setEventHandler(BLEDeviceEvent event, BLEDeviceEventHandler handler) {
  if (event >= 0 && event < BLEDeviceLastEvent) cur().ble.handlers[event] = handler;
}
void BLELocalDevice::setConnectable(bool connectable) { cur().ble.connectable = connectable; }
void BLELocalDevice::setTimeout(unsigned long timeoutMs) { cur().ble.timeoutMs = timeoutMs; }

// -------------------------------------------------------------- BLEDevice ---
bool BLEDevice::operator==(const BLEDevice& o) const {
  if (!s_ || !o.s_) return !s_ && !o.s_;
  return s_->viewer == o.s_->viewer && s_->peer == o.s_->peer;
}
void BLEDevice::poll(unsigned long timeout) { BLE.poll(timeout); }
bool BLEDevice::connected() const {
  if (!s_) return false;
  Connection* c = air().connection(s_->conn);
  if (!c) return false;
  return c->central == s_->viewer ? c->centralUp : c->peripheralUp;
}
bool BLEDevice::disconnect() {
  if (!s_) return false;
  Connection* c = air().connection(s_->conn);
  Board& b = cur();
  if (!c) return false;
  air().closeByLocal(*c, b, b.now());
  return true;
}
String BLEDevice::address() const { return s_ ? String(boardAddress(s_->peer)) : String(); }
bool BLEDevice::hasLocalName() const { return localName().length() > 0; }
String BLEDevice::localName() const {
  Board* p = s_ ? air().board(s_->peer) : nullptr;
  return p ? String(p->ble.localName) : String();
}
String BLEDevice::deviceName() {
  Board* p = s_ ? air().board(s_->peer) : nullptr;
  return p ? String(p->ble.deviceName) : String();
}
bool BLEDevice::hasAdvertisedServiceUuid() const { return advertisedServiceUuid().length() > 0; }
String BLEDevice::advertisedServiceUuid() const {
  Board* p = s_ ? air().board(s_->peer) : nullptr;
  return p ? String(p->ble.advertisedUuid) : String();
}
int BLEDevice::rssi() { return connected() ? -55 : 127; }

bool BLEDevice::connect() {
  if (!s_) return false;
  Board& b = cur();
  if (connected()) return true;
  Board* p = air().board(s_->peer);
  b.spend(air().blocking(air().config.connectUs));  // 在 ArduinoBLE 内部阻塞
  if (!p || !air().reachable(b, *p) || !p->ble.advertising || !p->ble.connectable) {
    air().note(b.now(), b.name + "：connect() 失败");
    return false;
  }
  Connection c;
  c.id = air().nextConn++;
  c.central = b.id;
  c.peripheral = p->id;
  air().connections.push_back(c);
  p->ble.advertising = false;
  s_->conn = c.id;
  s_->discovered.clear();
  air().post(*p, BleEvent{BleEvent::Connected, b.now(), c.id, "", {}});
  air().post(b, BleEvent{BleEvent::Connected, b.now(), c.id, "", {}});
  air().note(b.now(), "无线：" + b.name + " 已连接到 " + p->name + "（连接 " +
                          std::to_string(c.id) + "）");
  return true;
}

static bool linkAlive(Board& b, const DeviceState& d) {
  Connection* c = air().connection(d.conn);
  if (!c || !(c->central == b.id ? c->centralUp : c->peripheralUp)) return false;
  if (!c->physical) {  // 请求在 BLE 库内部超时
    b.spend(static_cast<uint64_t>(b.ble.timeoutMs) * 1000ULL);
    return false;
  }
  return true;
}

bool BLEDevice::discoverService(const char* serviceUuid) {
  if (!s_) return false;
  Board& b = cur();
  b.spend(air().blocking(air().config.discoverUs));
  if (!linkAlive(b, *s_)) return false;
  Board* p = air().board(s_->peer);
  bool found = false;
  const std::string wanted = serviceUuid ? lower(serviceUuid) : "";
  for (auto& svc : p->ble.services) {
    if (!wanted.empty() && svc->uuid != wanted) continue;
    auto copy = std::make_shared<ServiceState>();
    copy->uuid = svc->uuid;
    for (auto& ch : svc->chars) {
      auto rc = std::make_shared<CharState>();
      rc->uuid = ch->uuid;
      rc->props = ch->props;
      rc->capacity = ch->capacity;
      rc->value = ch->value;
      rc->owner = b.id;
      rc->remote = true;
      rc->conn = s_->conn;
      copy->chars.push_back(rc);
      air().remoteChars.push_back(rc);
    }
    s_->discovered.erase(std::remove_if(s_->discovered.begin(), s_->discovered.end(),
                                        [&](const std::shared_ptr<ServiceState>& s) {
                                          return s->uuid == copy->uuid;
                                        }),
                         s_->discovered.end());
    s_->discovered.push_back(copy);
    found = true;
  }
  return found;
}
bool BLEDevice::discoverAttributes() { return discoverService(nullptr); }
bool BLEDevice::hasService(const char* uuid) const { return static_cast<bool>(service(uuid)); }
int BLEDevice::serviceCount() const { return s_ ? static_cast<int>(s_->discovered.size()) : 0; }
BLEService BLEDevice::service(int index) const {
  if (!s_ || index < 0 || index >= serviceCount()) return BLEService();
  return BLEService(s_->discovered[static_cast<size_t>(index)]);
}
BLEService BLEDevice::service(const char* uuid) const {
  if (!s_ || !uuid) return BLEService();
  for (auto& s : s_->discovered)
    if (s->uuid == lower(uuid)) return BLEService(s);
  return BLEService();
}
int BLEDevice::characteristicCount() const {
  int n = 0;
  if (s_)
    for (auto& s : s_->discovered) n += static_cast<int>(s->chars.size());
  return n;
}
bool BLEDevice::hasCharacteristic(const char* uuid) const {
  return static_cast<bool>(characteristic(uuid));
}
BLECharacteristic BLEDevice::characteristic(int index) const {
  if (!s_) return BLECharacteristic();
  for (auto& s : s_->discovered)
    for (auto& c : s->chars)
      if (index-- == 0) return BLECharacteristic(c);
  return BLECharacteristic();
}
BLECharacteristic BLEDevice::characteristic(const char* uuid) const {
  if (!s_ || !uuid) return BLECharacteristic();
  for (auto& s : s_->discovered)
    for (auto& c : s->chars)
      if (c->uuid == lower(uuid)) return BLECharacteristic(c);
  return BLECharacteristic();
}

// ------------------------------------------------------------- BLEService ---
BLEService::BLEService() {}
BLEService::BLEService(const char* uuid) : s_(std::make_shared<ServiceState>()) {
  s_->uuid = lower(uuid ? uuid : "");
}
String BLEService::uuid() const { return s_ ? String(s_->uuid) : String(); }
void BLEService::addCharacteristic(BLECharacteristic& characteristic) {
  if (s_ && characteristic) s_->chars.push_back(characteristic.state());
}
int BLEService::characteristicCount() const { return s_ ? static_cast<int>(s_->chars.size()) : 0; }
bool BLEService::hasCharacteristic(const char* uuid) const { return static_cast<bool>(characteristic(uuid)); }
BLECharacteristic BLEService::characteristic(int index) const {
  if (!s_ || index < 0 || index >= characteristicCount()) return BLECharacteristic();
  return BLECharacteristic(s_->chars[static_cast<size_t>(index)]);
}
BLECharacteristic BLEService::characteristic(const char* uuid) const {
  if (!s_ || !uuid) return BLECharacteristic();
  for (auto& c : s_->chars)
    if (c->uuid == lower(uuid)) return BLECharacteristic(c);
  return BLECharacteristic();
}

// ------------------------------------------------------ BLECharacteristic ---
BLECharacteristic::BLECharacteristic() {}
BLECharacteristic::BLECharacteristic(const char* uuid, uint16_t properties, int valueSize,
                                     bool fixedLength)
    : s_(std::make_shared<CharState>()) {
  s_->uuid = lower(uuid ? uuid : "");
  s_->props = properties;
  s_->capacity = std::max(0, valueSize);
  s_->fixedLength = fixedLength;
  if (fixedLength) s_->value.assign(static_cast<size_t>(s_->capacity), 0);
}
BLECharacteristic::BLECharacteristic(const char* uuid, uint16_t properties, const char* value)
    : BLECharacteristic(uuid, properties, value ? static_cast<int>(strlen(value)) : 0) {
  if (value) s_->value.assign(value, value + strlen(value));
}
String BLECharacteristic::uuid() const { return s_ ? String(s_->uuid) : String(); }
uint8_t BLECharacteristic::properties() const { return s_ ? static_cast<uint8_t>(s_->props) : 0; }
int BLECharacteristic::valueSize() const { return s_ ? s_->capacity : 0; }
const uint8_t* BLECharacteristic::value() const {
  return s_ && !s_->value.empty() ? s_->value.data() : nullptr;
}
int BLECharacteristic::valueLength() const { return s_ ? static_cast<int>(s_->value.size()) : 0; }
uint8_t BLECharacteristic::operator[](int offset) const {
  return s_ && offset >= 0 && offset < valueLength() ? s_->value[static_cast<size_t>(offset)] : 0;
}

int BLECharacteristic::readValue(uint8_t value[], int length) {
  if (!s_ || !value || length <= 0) return 0;
  const int n = std::min(length, valueLength());
  if (n > 0) memcpy(value, s_->value.data(), static_cast<size_t>(n));
  return n;
}
int BLECharacteristic::readValue(void* value, int length) {
  return readValue(static_cast<uint8_t*>(value), length);
}
int BLECharacteristic::readValue(uint8_t& v) { return readValue(&v, sizeof v); }
int BLECharacteristic::readValue(int8_t& v) { return readValue(&v, sizeof v); }
int BLECharacteristic::readValue(uint16_t& v) { return readValue(&v, sizeof v); }
int BLECharacteristic::readValue(int16_t& v) { return readValue(&v, sizeof v); }
int BLECharacteristic::readValue(uint32_t& v) { return readValue(&v, sizeof v); }
int BLECharacteristic::readValue(int32_t& v) { return readValue(&v, sizeof v); }

int BLECharacteristic::writeValue(const uint8_t value[], int length, bool withResponse) {
  if (!s_ || length < 0 || (!value && length > 0)) return 0;
  Board& b = cur();
  std::vector<uint8_t> bytes(value, value + length);
  if (!s_->remote) {
    const size_t n = std::min(bytes.size(), static_cast<size_t>(s_->capacity));
    s_->value.assign(bytes.begin(), bytes.begin() + static_cast<long>(n));
    const int owner = s_->owner >= 0 ? s_->owner : b.id;
    if (s_->props & (BLENotify | BLEIndicate))
      for (auto& c : air().connections)
        if (c.peripheral == owner && c.peripheralUp &&
            std::find(c.subscribedUuids.begin(), c.subscribedUuids.end(), s_->uuid) !=
                c.subscribedUuids.end())
          air().transmit(c, false, s_->uuid, s_->value, BleEvent::Notify, b.now());
    return 1;
  }
  Connection* c = air().connection(s_->conn);
  if (!c || !c->centralUp) return 0;
  if (!(s_->props & (BLEWrite | BLEWriteWithoutResponse))) return 0;
  if (withResponse && (s_->props & BLEWrite)) {
    b.spend(air().blocking(air().config.attRoundTripUs));
    if (!c->physical) {
      b.spend(static_cast<uint64_t>(b.ble.timeoutMs) * 1000ULL);
      return 0;
    }
  }
  s_->value = bytes;
  air().transmit(*c, true, s_->uuid, bytes, BleEvent::Write, b.now());
  return 1;
}
int BLECharacteristic::writeValue(const void* value, int length, bool withResponse) {
  return writeValue(static_cast<const uint8_t*>(value), length, withResponse);
}
int BLECharacteristic::writeValue(const char* value, bool withResponse) {
  return writeValue(reinterpret_cast<const uint8_t*>(value),
                    value ? static_cast<int>(strlen(value)) : 0, withResponse);
}
int BLECharacteristic::writeValue(uint8_t v, bool r) { return writeValue(&v, sizeof v, r); }
int BLECharacteristic::writeValue(int8_t v, bool r) { return writeValue(&v, sizeof v, r); }
int BLECharacteristic::writeValue(uint16_t v, bool r) { return writeValue(&v, sizeof v, r); }
int BLECharacteristic::writeValue(int16_t v, bool r) { return writeValue(&v, sizeof v, r); }
int BLECharacteristic::writeValue(uint32_t v, bool r) { return writeValue(&v, sizeof v, r); }
int BLECharacteristic::writeValue(int32_t v, bool r) { return writeValue(&v, sizeof v, r); }

bool BLECharacteristic::written() {
  if (!s_) return false;
  const bool w = s_->writtenFlag;
  s_->writtenFlag = false;
  return w;
}
bool BLECharacteristic::subscribed() {
  if (!s_ || s_->remote) return s_ && s_->remoteSubscribed;
  for (auto& c : air().connections)
    if (c.peripheral == s_->owner && c.peripheralUp &&
        std::find(c.subscribedUuids.begin(), c.subscribedUuids.end(), s_->uuid) !=
            c.subscribedUuids.end())
      return true;
  return false;
}
bool BLECharacteristic::valueUpdated() {
  if (!s_) return false;
  Board& b = cur();
  b.spend(b.ioCallUs);
  const bool u = s_->updatedFlag;
  s_->updatedFlag = false;
  return u;
}
void BLECharacteristic::setEventHandler(int event, BLECharacteristicEventHandler handler) {
  if (s_ && event >= 0 && event < BLECharacteristicEventLast) s_->handlers[event] = handler;
}
bool BLECharacteristic::canRead() { return s_ && (s_->props & BLERead); }
bool BLECharacteristic::read() {
  if (!s_ || !s_->remote || !canRead()) return false;
  Board& b = cur();
  Connection* c = air().connection(s_->conn);
  if (!c || !c->centralUp) return false;
  b.spend(air().blocking(air().config.attRoundTripUs));
  if (!c->physical) {
    b.spend(static_cast<uint64_t>(b.ble.timeoutMs) * 1000ULL);
    return false;
  }
  Board* p = air().board(c->peripheral);
  auto local = p ? findLocalChar(*p, s_->uuid) : nullptr;
  if (local) s_->value = local->value;
  return true;
}
bool BLECharacteristic::canWrite() { return s_ && (s_->props & (BLEWrite | BLEWriteWithoutResponse)); }
bool BLECharacteristic::canSubscribe() { return s_ && (s_->props & (BLENotify | BLEIndicate)); }
bool BLECharacteristic::canUnsubscribe() { return canSubscribe(); }
bool BLECharacteristic::subscribe() {
  if (!s_ || !s_->remote || !canSubscribe()) return false;
  Board& b = cur();
  Connection* c = air().connection(s_->conn);
  if (!c || !c->centralUp) return false;
  b.spend(air().blocking(air().config.attRoundTripUs));
  if (!c->physical) {
    b.spend(static_cast<uint64_t>(b.ble.timeoutMs) * 1000ULL);
    return false;
  }
  if (std::find(c->subscribedUuids.begin(), c->subscribedUuids.end(), s_->uuid) ==
      c->subscribedUuids.end())
    c->subscribedUuids.push_back(s_->uuid);
  s_->remoteSubscribed = true;
  Board* p = air().board(c->peripheral);
  if (p) air().post(*p, BleEvent{BleEvent::Subscribed, b.now(), c->id, s_->uuid, {}});
  return true;
}
bool BLECharacteristic::unsubscribe() {
  if (!s_ || !s_->remote) return false;
  Connection* c = air().connection(s_->conn);
  if (c)
    c->subscribedUuids.erase(std::remove(c->subscribedUuids.begin(), c->subscribedUuids.end(),
                                         s_->uuid),
                             c->subscribedUuids.end());
  s_->remoteSubscribed = false;
  return true;
}
