#include <iostream>
#include <utility>
#include <random>
#include <iomanip>
#include <mutex>

// --------------- template STL for generator with poisson or cauchy distribution ------------
        template <typename E>
        class RandomPGenerator {
        public:
            using result_type = typename E::result_type;
            RandomPGenerator(result_type p) : _dist(p) {}
            result_type rand()
            {
                std::lock_guard<std::mutex> lock(_mutex);
                return _dist(_rgen);
            }

        private:
            E _dist;
            std::default_random_engine _rgen;
            std::mutex _mutex;
        };
        using rdist_poisson_t = std::poisson_distribution<int>;
        using rgen_poisson_t = RandomPGenerator<rdist_poisson_t>;

        template <typename F>
        class RandomCGenerator {
        public:
            using result_type = typename F::result_type;
            RandomCGenerator(result_type p, result_type j) : _dist(p, j) {}
            result_type rand()
            {
                std::lock_guard<std::mutex> lock(_mutex);
                return _dist(_rgen);
            }

        private:
            F _dist;
            std::default_random_engine _rgen;
            std::mutex _mutex;
        };
        using rdist_cauchy_t = std::cauchy_distribution<double>;
        using rgen_cauchy_t = RandomCGenerator<rdist_cauchy_t>;

int main() {

        // to use poisson distribution
        auto n = rgen_poisson_t(8.6);
        auto c = n.rand();
        std::cout << c << std::endl;

        // to use cauchy distribution
        auto l = rgen_cauchy_t(1,6);
        auto o = l.rand();
        std::cout << o << std::endl;
    
  return 0;    
 
}