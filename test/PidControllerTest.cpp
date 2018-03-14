#include <gtest/gtest.h>

#include "PidController.hpp"
#include "mock_arduino.hpp"
#include "unique_ptr.hpp"

using ::testing::Return;
using ::testing::StrictMock;
using ::testing::Test;

using namespace ArgoRcLib;
using namespace Libs;

namespace {
const unsigned long ONE_SEC = 1000;

unique_ptr<StrictMock<MockArduino>> createMockHardware() {

  unique_ptr<StrictMock<MockArduino>> hardware(new StrictMock<MockArduino>());

  EXPECT_CALL(*hardware, millis()).WillOnce(Return(ONE_SEC));
  EXPECT_CALL(*hardware, millis()).WillOnce(Return(0)).RetiresOnSaturation();
  return Libs::move(hardware);
}

class PidControllerFixture : public ::testing::Test {
protected:
  PidControllerFixture()
      : mockHardware(createMockHardware()), testInstance(*mockHardware) {}

  unique_ptr<StrictMock<MockArduino>> mockHardware;
  PidController testInstance;
};

TEST_F(PidControllerFixture, proportional) {
  // Set the current error to 1 meter - second
  const Distance oneMeter = 1_m;
  const Time oneSecond = 1_s;
  const Speed speedDelta(oneMeter, oneSecond);

  auto result = testInstance.calcProportional(speedDelta, oneSecond);
  (void)result;
}
} // namespace