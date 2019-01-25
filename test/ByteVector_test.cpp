#include "ByteVector.hpp"
#include <limits>
#include "gtest/gtest.h"

namespace msp {

// The fixture for testing class Foo.
template <typename T> class ByteVectorBasicTest : public ::testing::Test {};
typedef ::testing::Types<bool, uint8_t, uint16_t, uint32_t, int8_t, int16_t,
                         int32_t, float, double>
    basicTypes;
TYPED_TEST_CASE(ByteVectorBasicTest, basicTypes);

template <typename T> class ByteVectorScaledTest : public ::testing::Test {};
typedef ::testing::Types<uint8_t, uint16_t, uint32_t, int8_t, int16_t, int32_t>
    scaleOutputTypes;
TYPED_TEST_CASE(ByteVectorScaledTest, scaleOutputTypes);

// Tests that the Foo::Bar() method does Abc.
TEST(ByteVectorBasicTest, Initialzation) {
    ByteVector b;
    EXPECT_EQ(std::size_t(0), b.size());
    EXPECT_EQ(std::size_t(0), b.unpacking_offset());
    EXPECT_EQ(std::size_t(0), b.unpacking_remaining());
}

TYPED_TEST(ByteVectorBasicTest, Pack1zero) {
    ByteVector b;
    TypeParam ref = 0;
    EXPECT_TRUE(b.pack(ref));
    EXPECT_EQ(sizeof(TypeParam), b.size());
    EXPECT_TRUE(b.unpack(ref));
    EXPECT_EQ(TypeParam(0), ref);
}

TYPED_TEST(ByteVectorBasicTest, Pack1one) {
    ByteVector b;
    TypeParam ref = 1;
    EXPECT_TRUE(b.pack(ref));
    EXPECT_EQ(sizeof(TypeParam), b.size());
    EXPECT_TRUE(b.unpack(ref));
    EXPECT_EQ(TypeParam(1), ref);
}

TYPED_TEST(ByteVectorBasicTest, Pack1max) {
    ByteVector b;
    TypeParam ref = std::numeric_limits<TypeParam>::max();
    EXPECT_TRUE(b.pack(ref));
    EXPECT_EQ(sizeof(TypeParam), b.size());
    EXPECT_TRUE(b.unpack(ref));
    EXPECT_EQ(std::numeric_limits<TypeParam>::max(), ref);
}

TYPED_TEST(ByteVectorBasicTest, Pack1min) {
    ByteVector b;
    TypeParam ref = std::numeric_limits<TypeParam>::min();
    EXPECT_TRUE(b.pack(ref));
    EXPECT_EQ(sizeof(TypeParam), b.size());
    EXPECT_TRUE(b.unpack(ref));
    EXPECT_EQ(std::numeric_limits<TypeParam>::min(), ref);
}

TYPED_TEST(ByteVectorBasicTest, Pack1valzero) {
    ByteVector b;
    Value<TypeParam> v;
    TypeParam ref = 0;
    v             = ref;
    EXPECT_TRUE(b.pack(v));
    EXPECT_EQ(sizeof(TypeParam), b.size());
    EXPECT_TRUE(b.unpack(v));
    EXPECT_EQ(ref, v());
}

TYPED_TEST(ByteVectorBasicTest, Pack1valone) {
    ByteVector b;
    Value<TypeParam> v;
    TypeParam ref = 1;
    v             = ref;
    EXPECT_TRUE(b.pack(v));
    EXPECT_EQ(sizeof(TypeParam), b.size());
    EXPECT_TRUE(b.unpack(v));
    EXPECT_EQ(ref, v());
}

TYPED_TEST(ByteVectorBasicTest, Pack1valmax) {
    ByteVector b;
    Value<TypeParam> v;
    TypeParam ref = std::numeric_limits<TypeParam>::max();
    v             = ref;
    EXPECT_TRUE(b.pack(v));
    EXPECT_EQ(sizeof(TypeParam), b.size());
    EXPECT_TRUE(b.unpack(v));
    EXPECT_EQ(ref, v());
}

TYPED_TEST(ByteVectorBasicTest, Pack1valmin) {
    ByteVector b;
    Value<TypeParam> v;
    TypeParam ref = std::numeric_limits<TypeParam>::min();
    v             = ref;
    EXPECT_TRUE(b.pack(v));
    EXPECT_EQ(sizeof(TypeParam), b.size());
    EXPECT_TRUE(b.unpack(v));
    EXPECT_EQ(ref, v());
}

TYPED_TEST(ByteVectorBasicTest, Pack10zero) {
    ByteVector b;
    TypeParam ref = 0;
    for(int i = 0; i < 10; ++i) EXPECT_TRUE(b.pack(ref));
    EXPECT_EQ(10 * sizeof(TypeParam), b.size());
    for(int i = 0; i < 10; ++i) {
        EXPECT_TRUE(b.unpack(ref));
        EXPECT_EQ(TypeParam(0), ref);
    }
}

TYPED_TEST(ByteVectorBasicTest, Pack10one) {
    ByteVector b;
    TypeParam ref = 1;
    for(int i = 0; i < 10; ++i) EXPECT_TRUE(b.pack(ref));
    EXPECT_EQ(10 * sizeof(TypeParam), b.size());
    for(int i = 0; i < 10; ++i) {
        EXPECT_TRUE(b.unpack(ref));
        EXPECT_EQ(TypeParam(1), ref);
    }
}

TYPED_TEST(ByteVectorBasicTest, Pack10max) {
    ByteVector b;
    TypeParam ref = std::numeric_limits<TypeParam>::max();
    for(int i = 0; i < 10; ++i) EXPECT_TRUE(b.pack(ref));
    EXPECT_EQ(10 * sizeof(TypeParam), b.size());
    for(int i = 0; i < 10; ++i) {
        EXPECT_TRUE(b.unpack(ref));
        EXPECT_EQ(std::numeric_limits<TypeParam>::max(), ref);
    }
}

TYPED_TEST(ByteVectorBasicTest, Pack10min) {
    ByteVector b;
    TypeParam ref = std::numeric_limits<TypeParam>::min();
    for(int i = 0; i < 10; ++i) EXPECT_TRUE(b.pack(ref));
    EXPECT_EQ(10 * sizeof(TypeParam), b.size());
    for(int i = 0; i < 10; ++i) {
        EXPECT_TRUE(b.unpack(ref));
        EXPECT_EQ(std::numeric_limits<TypeParam>::min(), ref);
    }
}

TYPED_TEST(ByteVectorBasicTest, Pack10valzero) {
    ByteVector b;
    Value<TypeParam> v;
    TypeParam ref = 0;
    v             = ref;
    for(int i = 0; i < 10; ++i) EXPECT_TRUE(b.pack(v));
    EXPECT_EQ(10 * sizeof(TypeParam), b.size());
    for(int i = 0; i < 10; ++i) {
        EXPECT_TRUE(b.unpack(v));
        EXPECT_EQ(ref, v());
    }
}

TYPED_TEST(ByteVectorBasicTest, Pack10valone) {
    ByteVector b;
    Value<TypeParam> v;
    TypeParam ref = 1;
    v             = ref;
    for(int i = 0; i < 10; ++i) EXPECT_TRUE(b.pack(v));
    EXPECT_EQ(10 * sizeof(TypeParam), b.size());
    for(int i = 0; i < 10; ++i) {
        EXPECT_TRUE(b.unpack(v));
        EXPECT_EQ(ref, v());
    }
}

TYPED_TEST(ByteVectorBasicTest, Pack10valmax) {
    ByteVector b;
    Value<TypeParam> v;
    TypeParam ref = std::numeric_limits<TypeParam>::max();
    v             = ref;
    for(int i = 0; i < 10; ++i) EXPECT_TRUE(b.pack(v));
    EXPECT_EQ(10 * sizeof(TypeParam), b.size());
    for(int i = 0; i < 10; ++i) {
        EXPECT_TRUE(b.unpack(v));
        EXPECT_EQ(ref, v());
    }
}

TYPED_TEST(ByteVectorBasicTest, Pack10valmin) {
    ByteVector b;
    Value<TypeParam> v;
    TypeParam ref = std::numeric_limits<TypeParam>::min();
    v             = ref;
    for(int i = 0; i < 10; ++i) EXPECT_TRUE(b.pack(v));
    EXPECT_EQ(10 * sizeof(TypeParam), b.size());
    for(int i = 0; i < 10; ++i) {
        EXPECT_TRUE(b.unpack(v));
        EXPECT_EQ(ref, v());
    }
}

TYPED_TEST(ByteVectorScaledTest, Pack1DoubleZero) {
    ByteVector b;
    double ref = 0.0;
    EXPECT_TRUE(b.pack<TypeParam>(ref, 1));
    EXPECT_EQ(sizeof(TypeParam), b.size());
    EXPECT_TRUE(b.unpack<TypeParam>(ref, 1));
    EXPECT_DOUBLE_EQ(0, ref);
}

TYPED_TEST(ByteVectorScaledTest, Pack1DoubleTen) {
    ByteVector b;
    double ref = 10.0;
    EXPECT_TRUE(b.pack<TypeParam>(ref, 1));
    EXPECT_EQ(sizeof(TypeParam), b.size());
    EXPECT_TRUE(b.unpack<TypeParam>(ref, 1));
    EXPECT_DOUBLE_EQ(10.f, ref);
}

TYPED_TEST(ByteVectorScaledTest, Pack1DoubleSaturationMax) {
    ByteVector b;
    double ref = (double)std::numeric_limits<TypeParam>::max() + 1.0;
    EXPECT_TRUE(b.pack<TypeParam>(ref, 1));
    EXPECT_EQ(sizeof(TypeParam), b.size());
    EXPECT_TRUE(b.unpack<TypeParam>(ref, 1));
    EXPECT_DOUBLE_EQ(std::numeric_limits<TypeParam>::max(), ref);
}

TYPED_TEST(ByteVectorScaledTest, Pack1DoubleSaturationMin) {
    ByteVector b;
    double ref = (double)std::numeric_limits<TypeParam>::min() - 1.0;
    EXPECT_TRUE(b.pack<TypeParam>(ref, 1));
    EXPECT_EQ(sizeof(TypeParam), b.size());
    EXPECT_TRUE(b.unpack<TypeParam>(ref, 1));
    EXPECT_DOUBLE_EQ(std::numeric_limits<TypeParam>::min(), ref);
}

TYPED_TEST(ByteVectorScaledTest, Pack1FloatZero) {
    ByteVector b;
    float ref = 0.0;
    EXPECT_TRUE(b.pack<TypeParam>(ref, 1));
    EXPECT_EQ(sizeof(TypeParam), b.size());
    EXPECT_TRUE(b.unpack<TypeParam>(ref, 1));
    EXPECT_FLOAT_EQ(0.f, ref);
}

TYPED_TEST(ByteVectorScaledTest, Pack1FloatTen) {
    ByteVector b;
    float ref = 10.0;
    EXPECT_TRUE(b.pack<TypeParam>(ref, 1));
    EXPECT_EQ(sizeof(TypeParam), b.size());
    EXPECT_TRUE(b.unpack<TypeParam>(ref, 1));
    EXPECT_FLOAT_EQ(10.f, ref);
}

TYPED_TEST(ByteVectorScaledTest, Pack1FloatSaturationMax) {
    ByteVector b;
    float ref = (float)std::numeric_limits<TypeParam>::max() + 1.0;
    EXPECT_TRUE(b.pack<TypeParam>(ref, 1));
    EXPECT_EQ(sizeof(TypeParam), b.size());
    EXPECT_TRUE(b.unpack<TypeParam>(ref, 1));
    EXPECT_FLOAT_EQ(std::numeric_limits<TypeParam>::max(), ref);
}

TYPED_TEST(ByteVectorScaledTest, Pack1FloatSaturationMin) {
    ByteVector b;
    float ref = (float)std::numeric_limits<TypeParam>::min() - 1.0;
    EXPECT_TRUE(b.pack<TypeParam>(ref, 1));
    EXPECT_EQ(sizeof(TypeParam), b.size());
    EXPECT_TRUE(b.unpack<TypeParam>(ref, 1));
    EXPECT_FLOAT_EQ(std::numeric_limits<TypeParam>::min(), ref);
}

}  // namespace msp

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
