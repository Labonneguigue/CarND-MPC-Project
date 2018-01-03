#ifndef UTILS_H
#define UTILS_H

namespace utl {

    template <typename T>
    constexpr T pi()
    {
        return T(3.141592653589793);
    }

    template <typename T>
    inline T deg2rad(T x) {
        return x * pi<T>() / T(180.0);
    }

    template <typename T>
    T rad2deg(T x) {
        return x * T(180.0) / pi<T>();
    }
    
} // namespace utl

#endif //UTILS_H
