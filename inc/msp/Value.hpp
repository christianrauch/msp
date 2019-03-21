#ifndef VALUE_HPP
#define VALUE_HPP

#include <iostream>
#include <utility>

namespace msp {

template <class T> class Value {
public:
    /**
     * @brief Copy assignment operator
     * @returns Reference to this object
     */
    Value<T>& operator=(const Value<T>& rhs) {
        data.first  = rhs();
        data.second = rhs.set();
        return *this;
    }

    /**
     * @brief cast to the internal type
     */
    operator T() const { return data.first; }

    /**
     * @brief cast to the writable refernce of internal type
     */
    operator T&() { return data.first; }

    /**
     * @brief Assignment operator for non-Value objects
     * @returns Reference to this object
     */
    Value<T>& operator=(const T rhs) {
        data.first  = rhs;
        data.second = true;
        return *this;
    }

    /**
     * @brief Gets a reference to the internal data
     * @returns Reference to the data
     */
    T& operator()() { return data.first; }

    /**
     * @brief Gets a copy of the data
     * @returns
     */
    T operator()() const { return data.first; }

    /**
     * @brief Queries if the data has been set
     * @returns True if the internal data has been assigned
     */
    bool set() const { return data.second; }

    /**
     * @brief Gets a reference to the data valid flag
     * @returns Reference to the data valid flag
     */
    bool& set() { return data.second; }

private:
    std::pair<T, bool> data;
};

}  // namespace msp

template <class T>
inline std::ostream& operator<<(std::ostream& s, const msp::Value<T>& val) {
    if(val.set())
        s << val();
    else
        s << "<unset>";
    return s;
}

template <>
inline std::ostream& operator<<(std::ostream& s,
                                const msp::Value<uint8_t>& val) {
    if(val.set())
        s << uint32_t(val());
    else
        s << "<unset>";
    return s;
}

template <>
inline std::ostream& operator<<(std::ostream& s,
                                const msp::Value<int8_t>& val) {
    if(val.set())
        s << int32_t(val());
    else
        s << "<unset>";
    return s;
}

#endif
