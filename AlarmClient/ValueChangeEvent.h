#ifndef ValueChangeEvent_h
#define ValueChangeEvent_h

typedef unsigned char byte;

class ValueChangeEvent {
  public:
  bool isSet;
  byte value;

  ValueChangeEvent();
  ValueChangeEvent(byte value);
};

#endif
