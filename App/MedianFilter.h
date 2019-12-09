#pragma once


#define MedianFilterLen 15
class MedianFilter
{
public:

  MedianFilter() {
    reset();
  }

  void reset() {
    for (uint8_t i = 0; i < MedianFilterLen; i++)
    {
      dataBuffer_[i] = { 0, i };
    }
    time_ = 0;
  }

  int getOldestPos() {
    uint8_t oldest_time = (uint8_t)(MedianFilterLen + time_ - MedianFilterLen) % MedianFilterLen;
    for (int i = 0; i < MedianFilterLen; i++) {
      if (dataBuffer_[i].time == oldest_time)
        return i;
    }
    return 0;
  }

  int16_t compute(int16_t val)
  {
    int pos = getOldestPos();
    dataBuffer_[pos].val = val;
    dataBuffer_[pos].time = time_++;
    if (time_ >= MedianFilterLen)
      time_ = 0;

    while (pos != 0 && val < dataBuffer_[pos - 1].val) {
      ValAndTime tmp = dataBuffer_[pos - 1];
      dataBuffer_[pos - 1] = dataBuffer_[pos];
      dataBuffer_[pos] = tmp;
      pos--;
    }

    while (pos < MedianFilterLen - 1 && val > dataBuffer_[pos + 1].val) {
      ValAndTime tmp = dataBuffer_[pos + 1];
      dataBuffer_[pos + 1] = dataBuffer_[pos];
      dataBuffer_[pos] = tmp;
      pos++;
    }

    return dataBuffer_[MedianFilterLen / 2].val;
  }

private:

  struct ValAndTime {
    int16_t val;
    uint8_t time;
  };

  ValAndTime dataBuffer_[MedianFilterLen];
  uint8_t time_ = 0;
};
