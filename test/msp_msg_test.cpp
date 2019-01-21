#include "gtest/gtest.h"
#include "msp_msg_maker.hpp"

namespace msp {

// The fixture for testing class Foo.

class MspMsgTest : public ::testing::Test {
public:
    MspMsgTest() : msg_maker(FirmwareVariant::INAV){};

    message_maker msg_maker;
};

// Tests that the Foo::Bar() method does Abc.
TEST_F(MspMsgTest, Initialzation) {
    auto m = msg_maker(ID::MSP_STATUS);
    ASSERT_TRUE(bool(m));

    auto b = m->encode();
    ASSERT_TRUE(bool(b));
    ASSERT_TRUE(m->decode(*b));
}

}  // namespace msp

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
