#include "ValueChangeEvent.h"

ValueChangeEvent::ValueChangeEvent() {
  isSet = false;
  value = 0;
}

ValueChangeEvent::ValueChangeEvent(byte value) {
  this->value = value;
}
