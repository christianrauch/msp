#include "Value.hpp"
#include <type_traits>
#include "ByteVector.hpp"
#include "gtest/gtest.h"

namespace msp {

// The fixture for testing class Foo.
template <typename T> class valueTest : public ::testing::Test {
protected:
    // You can remove any or all of the following functions if its body
    // is empty.

    valueTest() {
        // You can do set-up work for each test here.
    }

    virtual ~valueTest() {
        // You can do clean-up work that doesn't throw exceptions here.
    }

    // If the constructor and destructor are not enough for setting up
    // and cleaning up each test, you can define the following methods:

    virtual void SetUp() {
        // Code here will be called immediately after the constructor (right
        // before each test).
    }

    virtual void TearDown() {
        // Code here will be called immediately after each test (right
        // before the destructor).
    }

    // Objects declared here can be used by all tests in the test case for Foo.
};

typedef ::testing::Types<bool, uint8_t, uint16_t, uint32_t, int8_t, int16_t,
                         int32_t, float, double>
    numTypes;
TYPED_TEST_CASE(valueTest, numTypes);

// Tests that the Foo::Bar() method does Abc.
TYPED_TEST(valueTest, Initialzation) {
    Value<TypeParam> v;
    EXPECT_EQ(TypeParam(0), v());
    EXPECT_FALSE(v.set());
}

TYPED_TEST(valueTest, AssignmentZero) {
    Value<TypeParam> v1, v2;

    TypeParam ref = 0;
    v1            = ref;
    EXPECT_EQ(ref, v1());
    EXPECT_TRUE(v1.set());
    v2 = v1;
    EXPECT_EQ(ref, v2());
    EXPECT_TRUE(v2.set());
}

TYPED_TEST(valueTest, AssignmentMax) {
    Value<TypeParam> v1, v2;

    TypeParam ref = std::numeric_limits<TypeParam>::max();
    ;
    v1 = ref;
    EXPECT_EQ(ref, v1());
    EXPECT_TRUE(v1.set());
    v2 = v1;
    EXPECT_EQ(ref, v2());
    EXPECT_TRUE(v2.set());
}

TYPED_TEST(valueTest, AssignmentMin) {
    Value<TypeParam> v1, v2;

    TypeParam ref = std::numeric_limits<TypeParam>::min();
    ;
    v1 = ref;
    EXPECT_EQ(ref, v1());
    EXPECT_TRUE(v1.set());
    v2 = v1;
    EXPECT_EQ(ref, v2());
    EXPECT_TRUE(v2.set());
}

TYPED_TEST(valueTest, AssignmentCast) {
    Value<TypeParam> v;
    EXPECT_FALSE(v.set());

    const TypeParam ref1 = std::numeric_limits<TypeParam>::max();

    v = ref1;
    EXPECT_TRUE(v.set());

    EXPECT_EQ(ref1, v());

    const TypeParam ref2 = v;
    EXPECT_EQ(ref1, ref2);
}

TEST(valueTest, stringInit) {
    Value<std::string> v;
    EXPECT_EQ("", v());
    EXPECT_EQ(false, v.set());
}

TEST(valueTest, stringAssign) {
    Value<std::string> v1, v2;
    v1 = std::string("test");
    EXPECT_EQ("test", v1());
    EXPECT_EQ(true, v1.set());
    v2 = v1;
    EXPECT_EQ("test", v2());
    EXPECT_EQ(true, v2.set());
}

TEST(valueTest, ByteVecInit) {
    Value<ByteVector> v;
    EXPECT_EQ(true, v().empty());
    EXPECT_EQ(false, v.set());
}

TEST(valueTest, ByteVecAssign) {
    Value<ByteVector> v1, v2;
    v1 = ByteVector(1, 1);
    EXPECT_EQ(1, v1()[0]);
    EXPECT_EQ(true, v1.set());
    v2 = v1;
    EXPECT_EQ(1, v2()[0]);
    EXPECT_EQ(true, v2.set());
}

}  // namespace msp

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
