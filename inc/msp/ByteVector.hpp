#ifndef BYTE_VECTOR_HPP
#define BYTE_VECTOR_HPP

#include <cstdint>
#include <limits>
#include <memory>
#include <type_traits>
#include <vector>
#include "Value.hpp"

namespace msp {

struct Packable;

class ByteVector : public std::vector<uint8_t> {
public:
    /**
     * @brief ByteVector constructor
     */
    ByteVector() : offset(0) {}

    /**
     * @brief ByteVector constructor
     * @param arg1 Single argument passed to constructor of parent std::vector
     * class
     */
    template <typename T1>
    ByteVector(T1 arg1) : std::vector<uint8_t>(arg1), offset(0) {}

    /**
     * @brief ByteVector constructor
     * @param arg1 First of two arguments passed to constructor of parent
     * std::vector class
     * @param arg2 Second of two arguments passed to constructor of parent
     * std::vector class
     */
    template <typename T1, typename T2>
    ByteVector(T1 arg1, T2 arg2) :
        std::vector<uint8_t>(arg1, arg2),
        offset(0) {}

    /**
     * @brief Packs integer types into the ByteVector. Ensures little endian
     * packing.
     * @tparam T Underlying data type to be packed. Must be an integral type.
     * @param val Value to be packed
     * @return true
     */
    template <typename T, typename std::enable_if<std::is_integral<T>::value,
                                                  T>::type* = nullptr>
    bool pack(const T& val) {
        // little endian packing
        for(size_t i(0); i < sizeof(val); ++i) {
            this->push_back(val >> (i * 8) & 0xFF);
        }
        return true;
    }

    /**
     * @brief Packs floating point types into the ByteVector. Uses IEEE 754
     * format.
     * @tparam T Underlying data type to be packed. Must be a floating point
     * type.
     * @param val Value to be packed
     * @return True if successful
     */
    template <typename T,
              typename std::enable_if<std::is_floating_point<T>::value,
                                      T>::type* = nullptr>
    bool pack(const T& val) {
        ByteVector b(sizeof(val));
        reinterpret_cast<T&>(b.front()) = val;
        return this->pack(b);
    }

    /**
     * @brief Packs scaled values (e.g. float to scaled int) as packed_val =
     * (val+offset)*scale
     * @tparam encoding_T Data type to use for the actual data packing (usually
     * an integral type)
     * @tparam T1 Type of input value (usually a floating point type)
     * @tparam T2 Type of scale and offset coefficients
     * @param val Value to be packed
     * @param scale Value of scaling to apply to the offset value
     * @param offset Value of offset to apply to the input value (optional,
     * defaults to 0)
     * @return True if successful
     */
    template <typename encoding_T, typename T1, typename T2,
              typename std::enable_if<std::is_arithmetic<T1>::value,
                                      T1>::type* = nullptr,
              typename std::enable_if<std::is_arithmetic<T2>::value,
                                      T2>::type* = nullptr>
    bool pack(const T1 val, const T2 scale, const T2 offset = 0) {
        auto tmp = (val + offset) * scale;
        if(tmp <= std::numeric_limits<encoding_T>::min())
            return pack(std::numeric_limits<encoding_T>::min());
        else if(tmp >= std::numeric_limits<encoding_T>::max())
            return pack(std::numeric_limits<encoding_T>::max());
        return pack(static_cast<encoding_T>(tmp));
    }

    /**
     * @brief Packs string data into the ByteVector.
     * @param val String to be packed
     * @param max_len Optional max number of characters to transfer into the
     * ByteVector
     * @return True if successful
     */
    bool pack(const std::string& val,
              size_t max_len = std::numeric_limits<size_t>::max()) {
        size_t count = 0;
        for(auto c : val) {
            this->push_back(c);
            if(++count == max_len) break;
        }
        // TODO: validate that this null termination is the right thing to do in
        // all cases definitly correct in tested cases
        this->push_back(0);
        return true;
    }

    /**
     * @brief Packs another ByteVector into the ByteVector.
     * @param data ByteVector to be packed
     * @param max_len Optional max number of characters to transfer into the
     * ByteVector
     * @return True if successful
     */
    bool pack(const ByteVector& data,
              size_t max_len = std::numeric_limits<size_t>::max()) {
        size_t count = 0;
        for(auto c : data) {
            this->push_back(c);
            if(++count == max_len) break;
        }
        return true;
    }

    /**
     * @brief Packs an an object which inherits from type Packable into the
     * ByteVector
     * @param val Reference to object to be packed
     * @return True if successful
     */
    template <typename T,
              typename std::enable_if<std::is_base_of<Packable, T>::value,
                                      T>::type* = nullptr>
    bool pack(const T& val) {
        return val.pack_into(*this);
    }

    /**
     * @brief Packs scaled value types (e.g. value<float> to scaled int) as
     * packed_val = (val+offset)*scale
     * @tparam encoding_T Data type to use for the actual data packing (usually
     * an integral type)
     * @tparam T1 Type of input value (usually a floating point type)
     * @tparam T2 Type of scale and offset coefficients
     * @param val Value to be packed
     * @param scale Value of scaling to apply to the offset value
     * @param offset Value of offset to apply to the input value (optional,
     * defaults to 0)
     * @return True if successful
     */
    template <typename encoding_T, typename T1, typename T2,
              typename std::enable_if<std::is_arithmetic<T1>::value,
                                      T1>::type* = nullptr,
              typename std::enable_if<std::is_arithmetic<T2>::value,
                                      T2>::type* = nullptr>
    bool pack(const Value<T1> val, const T2 scale, const T2 offset = 0) {
        if(!val.set()) return false;
        return pack<encoding_T>(val(), scale, offset);
    }

    /**
     * @brief Packs a Value<ByteVector> into the ByteVector.
     * @param val The Value<ByteVector> to be packed
     * @param max_len Optional max number of characters to transfer into the
     * ByteVector
     * @return True if successful
     */
    bool pack(const Value<ByteVector>& val,
              size_t max_len = std::numeric_limits<size_t>::max()) {
        if(!val.set()) return false;
        return pack(val(), max_len);
    }

    /**
     * @brief Packs a Value<std::string> into the ByteVector.
     * @param val The Value<std::string> to be packed
     * @param max_len Optional max number of characters to transfer into the
     * ByteVector
     * @return True if successful
     */
    bool pack(const Value<std::string>& val,
              size_t max_len = std::numeric_limits<size_t>::max()) {
        if(!val.set()) return false;
        return pack(val(), max_len);
    }

    /**
     * @brief Packs the contents of a Value<T> into the ByteVector.
     * @tparam T Type of the Value<T> being packed. May be automatically deduced
     * from arguments
     * @param val The Value<T> to be packed
     * @return True if successful
     */
    template <class T> bool pack(const Value<T>& val) {
        if(!val.set()) return false;
        return pack(val());
    }

    /**
     * @brief Extracts little endian integers from the ByteVector. Consumes
     * a number of bytes matching sizeof(T). Fails if not enough bytes
     * are available.
     * @tparam T Underlying data type to be extracted. Must be an integral type.
     * @param val Destination of unpack operation.
     * @return True on successful unpack
     */
    template <typename T, typename std::enable_if<std::is_integral<T>::value,
                                                  T>::type* = nullptr>
    bool unpack(T& val) const {
        if(unpacking_remaining() < sizeof(val)) return false;
        val = 0;
        for(size_t i(0); i < sizeof(val); ++i) {
            val |= (*this)[offset++] << (8 * i);
        }
        return true;
    }

    /**
     * @brief unpack Extracts a boolen from a single byte
     * @param val Destination of unpack operation.
     * @return True on successful unpack
     */
    bool unpack(bool& val) const {
        if(unpacking_remaining() < 1) return false;
        val = (*this)[offset++];
        return true;
    }

    /**
     * @brief Extracts floating point numbers from the ByteVector.
     * Consumes a number of bytes matching sizeof(T). Fails if not enough
     * bytes are available.
     * @tparam T Underlying data type to be extracted. Must be a floating point
     * type.
     * @param val Destination of unpack operation.
     * @return True on successful unpack
     */
    template <typename T,
              typename std::enable_if<std::is_floating_point<T>::value,
                                      T>::type* = nullptr>
    bool unpack(T& val) const {
        if(unpacking_remaining() < sizeof(val)) return false;
        val = reinterpret_cast<const T&>((*this)[offset]);
        offset += sizeof(val);
        return true;
    }

    /**
     * @brief Extracts data from the ByteVector and stores it in a
     * std::string. Consumes all remaining data unless instructed otherwise.
     * @param val Destination of unpack operation.
     * @param count Max number of bytes to extract. Optional, if unset, all
     * remaining bytes will be consumed.
     * @return True on successful unpack
     */
    bool unpack(std::string& val,
                size_t count = std::numeric_limits<size_t>::max()) const {
        if(count == std::numeric_limits<size_t>::max())
            count = unpacking_remaining();
        if(count > unpacking_remaining()) return false;
        bool rc = true;
        val.clear();
        int8_t tmp;
        for(size_t i = 0; i < count; ++i) {
            rc &= unpack(tmp);
            val += tmp;
        }
        return rc;
    }

    /**
     * @brief Extracts data from the ByteVector and stores it in a
     * another ByteVector. Consumes all remaining data unless instructed
     * otherwise.
     * @param val Destination of unpack operation.
     * @param count Max number of bytes to extract. Optional, if unset, all
     * remaining bytes will be consumed.
     * @return True on successful unpack
     */
    bool unpack(ByteVector& val,
                size_t count = std::numeric_limits<size_t>::max()) const {
        if(count == std::numeric_limits<size_t>::max())
            count = unpacking_remaining();
        if(!consume(count)) return false;
        val.clear();
        val.insert(
            val.end(), unpacking_iterator(), unpacking_iterator() + count);
        return true;
    }

    /**
     * @brief Unpacks scaled value types (e.g. scaled int to floating point) as
     * val = (packed_val/scale)-offset
     * @tparam encoding_T data type used to store the scaled value (usually an
     * integral type)
     * @tparam T1 type of output value (usually a floating point type)
     * @tparam T2 type of scale and offset coefficients
     * @param val Destination of unpack operation
     * @param scale Value of scaling to apply to the offset value
     * @param offset Value of offset to apply to the input value (optional,
     * defaults to 0)
     * @return True if successful
     */
    template <typename encoding_T, typename T1, typename T2,
              typename std::enable_if<std::is_arithmetic<T1>::value,
                                      T1>::type* = nullptr,
              typename std::enable_if<std::is_arithmetic<T2>::value,
                                      T2>::type* = nullptr>
    bool unpack(T1& val, T2 scale, T2 offset = 0) const {
        bool rc        = true;
        encoding_T tmp = 0;
        rc &= unpack(tmp);
        val = tmp / scale;
        val -= offset;
        return rc;
    }

    /**
     * @brief unpack Unpacks an an object which inherits from type Packable
     * @param val Reference to object to be unpacked
     * @return True if successful
     */
    template <typename T,
              typename std::enable_if<std::is_base_of<Packable, T>::value,
                                      T>::type* = nullptr>
    bool unpack(T& obj) const {
        return obj.unpack_from(*this);
    }

    /**
     * @brief Unpacks Value types other than string and ByteVector
     * specializations
     * @tparam T Type of the Value<T> being packed. May be automatically deduced
     * from arguments
     * @param val The destination of the unpack operation
     * @return  true on success
     */
    template <class T> bool unpack(Value<T>& val) const {
        return val.set() = unpack(val());
    }

    /**
     * @brief Extracts data from the ByteVector and stores it in a
     * Value<std::string>. Consumes all remaining data unless instructed
     * otherwise.
     * @param val Destination of unpack operation.
     * @param count Max number of bytes to extract. Optional, if unset, all
     * remaining bytes will be consumed.
     * @return True on successful unpack
     */
    bool unpack(Value<std::string>& val,
                size_t count = std::numeric_limits<size_t>::max()) const {
        return val.set() = unpack(val(), count);
    }

    /**
     * @brief Extracts data from the ByteVector and stores it in a
     * Value<ByteVector>. Consumes all remaining data unless instructed
     * otherwise.
     * @param val Destination of unpack operation.
     * @param count Max number of bytes to extract. Optional, if unset, all
     * remaining bytes will be consumed.
     * @return True on successful unpack
     */
    bool unpack(Value<ByteVector>& val,
                size_t count = std::numeric_limits<size_t>::max()) const {
        return val.set() = unpack(val(), count);
    }

    /**
     * @brief Unpacks scaled Value types (e.g. scaled int to floating point) as
     * val = (packed_val/scale)-offset
     * @tparam encoding_T data type used to store the scaled Value (usually an
     * integral type)
     * @tparam T1 type of output Value (usually a floating point type)
     * @tparam T2 type of scale and offset coefficients
     * @param val Destination of unpack operation
     * @param scale Value of scaling to apply to the offset value
     * @param offset Value of offset to apply to the input value (optional,
     * defaults to 0)
     * @return True if successful
     */
    template <typename encoding_T, typename T1, typename T2,
              typename std::enable_if<std::is_arithmetic<T1>::value,
                                      T1>::type* = nullptr,
              typename std::enable_if<std::is_arithmetic<T2>::value,
                                      T2>::type* = nullptr>
    bool unpack(Value<T1>& val, T2 scale = 1, T2 offset = 0) const {
        return val.set() = unpack<encoding_T>(val(), scale, offset);
    }

    /**
     * @brief Gives the number of bytes which have already been consumed by
     * unpack operations.
     * @returns Number of bytes already consumed
     */
    std::size_t unpacking_offset() const { return offset; }

    /**
     * @brief Gives an iterator to the next element ready for unpacking
     * @returns iterator to the next byte for unpacking
     */
    std::vector<uint8_t>::iterator unpacking_iterator() {
        return this->begin() + offset;
    }

    /**
     * @brief Gives an iterator to the next element ready for unpacking
     * @returns iterator to the next byte for unpacking
     */
    std::vector<uint8_t>::const_iterator unpacking_iterator() const {
        return this->begin() + offset;
    }

    /**
     * @brief Manually consumes data, thus skipping the values.
     * @param count Number of bytes to consume
     * @returns True if successful
     * @returns False if there were not enough bytes to satisfy the request
     */
    bool consume(std::size_t count) const {
        if(count > unpacking_remaining()) {
            return false;
        }
        offset += count;
        return true;
    }

    /**
     * @brief Returns the number of bytes still avialable for unpacking
     * @returns Number of bytes remaining
     */
    std::size_t unpacking_remaining() const { return this->size() - offset; }

protected:
    mutable std::size_t offset;
};

/**
 * @brief Definition of a pure virtual class used to indicate that a child
 * class can pack itself into a ByteVector and unpack itself from a ByteVector.
 */
struct Packable {
    virtual ~Packable() {}
    virtual bool pack_into(ByteVector& data) const   = 0;
    virtual bool unpack_from(const ByteVector& data) = 0;
};

typedef std::shared_ptr<ByteVector> ByteVectorPtr;
typedef std::unique_ptr<ByteVector> ByteVectorUptr;

}  // namespace msp

inline std::ostream& operator<<(std::ostream& s, const msp::ByteVector& val) {
    s << std::hex;
    for(const auto& v : val) {
        s << uint32_t(v) << " ";
    }
    s << std::dec << std::endl;
    return s;
}

#endif
