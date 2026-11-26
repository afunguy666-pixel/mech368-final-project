#ifndef FWSIM_HCI_H
#define FWSIM_HCI_H
class HostHci {
 public:
  int availableAclPackets() const;
};
extern HostHci HCI;
#endif
