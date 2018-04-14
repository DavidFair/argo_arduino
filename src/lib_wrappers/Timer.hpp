#ifndef TIMER_HPP_
#define TIMER_HPP_

namespace Libs {

class Timer {
public:
  Timer(unsigned long timeBetweenFiring, unsigned long currentTime)
      : m_startingTime(currentTime), m_timeInterval(timeBetweenFiring) {}

  unsigned long getTimeDiff(unsigned long currentTime) const {
    return currentTime - m_startingTime;
  }

  bool hasTimerFired(unsigned long currentTime) const {
    return (currentTime - m_startingTime) > m_timeInterval;
  }

  void reset(unsigned long currentTime) { m_startingTime = currentTime; }

private:
  unsigned long m_startingTime;
  const unsigned long m_timeInterval;
};
} // namespace Libs

#endif // TIMER_HPP_