#include <iostream>
#include <utility>
#include <random>
#include <iomanip>
#include <mutex>

// --------------- template STL to generate random number of the type specified ------------
        template <typename D>
        class RandomGenerator {
        public:
            using result_type = typename D::result_type;
            RandomGenerator(result_type min, result_type max) : _dist(min, max), _rgen(std::random_device()()) {}
            result_type rand()
            {
                std::lock_guard<std::mutex> lock(_mutex);
                return _dist(_rgen);
            }

        private:
            D _dist;
            std::mt19937 _rgen;
            std::mutex _mutex;
        };
        using rdist_double_t = std::uniform_real_distribution<double>;
        using rdist_float_t = std::uniform_real_distribution<float>;
        using rdist_int_t = std::uniform_int_distribution<int>;
        using rdist_long_t = std::uniform_int_distribution<long>;
        using rdist_longlong_t = std::uniform_int_distribution<long>;

        using rgen_double_t = RandomGenerator<rdist_double_t>;
        using rgen_float_t = RandomGenerator<rdist_float_t>;
        using rgen_int_t = RandomGenerator<rdist_int_t>;
        using rgen_long_t = RandomGenerator<rdist_long_t>;
        using rgen_longlong_t = RandomGenerator<rdist_longlong_t>;

int main() {
        // to use a double generator
        auto x = rgen_double_t(0.0, 1.0);
        auto t = x.rand();
        std::cout << t << std::endl;

        // to use a integer generator
        auto z = rgen_int_t(0.0, 100.0);
        auto w = z.rand();
        std::cout << w << std::endl;

        // to use a float generator
        auto y = rgen_float_t(0.0, 10.0);
        auto m = y.rand();
        std::cout << m << std::endl;

        // to use a long generator
        auto a = rgen_long_t(0.0, 69770.0);
        auto s = a.rand();
        std::cout << s << std::endl;

        // to use a long long generator
        auto g = rgen_longlong_t(0.0, 1000000.0);
        auto b = g.rand();
        std::cout << b << std::endl;
  return 0;    
 

}
