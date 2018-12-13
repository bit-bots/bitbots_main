/*
This code is largely based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#ifndef COMBINATION_HPP
#define COMBINATION_HPP

#include <map>
#include <vector>

namespace bitbots_splines {

/**
 * Combination
 *
 * Implement binomial coefficient
 * computation using Pascal Triangle 
 * and iterate thought all (n choose k)
 * combinations
 */
class Combination
{
    public:

        /**
         * Compute the number of possible 
         * combinations for (n choose k)
         * (using dynamic progamming)
         */
        unsigned long binomialCoefficient(size_t k, size_t n);

        /**
         * Start combination iteration 
         * for given (n choose k)
         */
        void startCombination(size_t k, size_t n);

        /**
         * Return the next combination.
         * Return empty sdt::vector when
         * iteration is finished
         */
        std::vector<size_t> nextCombination();

    private:
        
        /**
         * Typedefs
         */
        typedef std::pair<unsigned long, unsigned long> Pair;
        typedef std::vector<size_t> Comb;

        /**
         * Hold (n choose k) number of possible 
         * combinations for dynamic programming
         */
        std::map<Pair, unsigned long> _pascalTriangle;

        /**
         * Current indexes container and
         * iteration n and k parameter
         */
        std::vector<size_t> _indexes;
        size_t _n;
        size_t _k;

        /**
         * Increment by one the _indexes container
         * at digit i (recursively).
         * Return true on iteration end
         */
        bool incrIndexes(size_t i);
};

}

#endif

