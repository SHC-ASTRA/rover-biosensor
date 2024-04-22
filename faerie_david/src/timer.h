#include <Arduino.h>
#include <vector>

typedef void (*callback_function)(void); // From https://stackoverflow.com/a/2582187


class Timer {
  public:
    unsigned interval;
    unsigned prevTime;
    String state;
    String name;
    callback_function func;

    Timer() {
      name = "unnamed";
      interval = 1000;
    }

    Timer(String inputName, unsigned inputInterval, unsigned currTime, std::vector<Timer>* timers, callback_function pFunc) {
      name = inputName;
      interval = inputInterval;
      prevTime = currTime;
      func = pFunc;
      timers->push_back(*this);
    }

    /*unsigned getPrevTime() {
      return prevTime;
    }*/

    unsigned skipNext() {
      prevTime += interval;
      return prevTime;
    }

  //private:
  //  unsigned prevTime;
};