#ifndef VALUE_HPP
#define VALUE_HPP

#include <iostream>
#include <utility>

namespace msp {

template<class T>
class value {
public:
    value<T>& operator= (value<T>& rhs) {
        data.first = rhs();
        data.second = rhs.set();
        return *this;
    }
    /*
    value<T>& operator= (T& rhs) {
        data.first = rhs;
        data.second = true;
        return *this;
    }
    */
    value<T>& operator= (T rhs) {
        data.first = rhs;
        data.second = true;
        return *this;
    }
    
    T& operator() () {
        return data.first;
    }
    
    T operator() () const {
        return data.first;
    }
    
    bool set() const {
        return data.second;
    }
    
    bool& set() {
        return data.second;
    }
    
    operator bool () {
        return data.second;
    }
    
private:
    std::pair<T,bool> data;
};


}


template<class T>
std::ostream& operator<<(std::ostream& s, const msp::value<T>& val) {
    if ( val.set() )
        s << val();
    else
        s << "<unset>";
    return s;
};

template<>
std::ostream& operator<<(std::ostream& s, const msp::value<uint8_t>& val) {
    if ( val.set() )
        s << uint32_t(val());
    else
        s << "<unset>";
    return s;
};

template<>
std::ostream& operator<<(std::ostream& s, const msp::value<int8_t>& val) {
    if ( val.set() )
        s << int32_t(val());
    else
        s << "<unset>";
    return s;
};

#endif
