#ifndef ValueChangeEventQueue_h
#define ValueChangeEventQueue_h
#include "ValueChangeEvent.h"

class ValueChangeEventQueue {
  private:
  int eventQueueSize;
  ValueChangeEvent *eventQueue;
  ValueChangeEvent *currentEvent;
  ValueChangeEvent *eventQueueFirst;
  ValueChangeEvent *eventQueueLast;

  public:
  ValueChangeEventQueue(int eventQueueSize);
  ~ValueChangeEventQueue();
  
  void moveToNext(ValueChangeEvent **ppEvent);
  void addNewEvent(const ValueChangeEvent &newEvent);
  ValueChangeEvent* getNextEvent();
};

#endif
