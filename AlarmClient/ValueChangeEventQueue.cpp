#include "ValueChangeEventQueue.h"

ValueChangeEventQueue::ValueChangeEventQueue(int eventQueueSize) {
  this->eventQueueSize = eventQueueSize;

  eventQueue = new ValueChangeEvent[eventQueueSize];
  
  eventQueueFirst = eventQueue;
  eventQueueLast = eventQueue + (eventQueueSize - 1);
  currentEvent = eventQueueFirst;
}

ValueChangeEventQueue::~ValueChangeEventQueue() {
  delete [] eventQueue;
}

void ValueChangeEventQueue::moveToNext(ValueChangeEvent **ppEvent) {
  if (*ppEvent == eventQueueLast)
    *ppEvent = eventQueueFirst;
  else
    *ppEvent = *ppEvent + 1;
}

void ValueChangeEventQueue::addNewEvent(const ValueChangeEvent &newEvent) {
  ValueChangeEvent *event = currentEvent;

  while (event->isSet) {
    moveToNext(&event);
      
    if (event == currentEvent)
      break;
  }

  if (event->isSet) {
    moveToNext(&currentEvent);
  }

  *event = newEvent;
  event->isSet = true;
}

ValueChangeEvent* ValueChangeEventQueue::getNextEvent() {
  ValueChangeEvent *event = currentEvent;

  if(!event->isSet)
    return 0;

  event->isSet = false;
  moveToNext(&currentEvent);

  return event;
}

//void ValueChangeEventQueue::printEventQueue() {
//  int i=0;
//  while(i < eventQueueSize) {
//    ValueChangeEvent *event = &eventQueue[i];
//    
//    Serial.print(i);
//    Serial.print(": ");
//    if (event->isSet)
//      Serial.print(event->text);
//    else
//      Serial.print("EMPTY");
//
//    if (event == currentEvent)
//      Serial.print(" --");
//    Serial.println();
//    ++i;
//  }
//
//  Serial.println();
//}
