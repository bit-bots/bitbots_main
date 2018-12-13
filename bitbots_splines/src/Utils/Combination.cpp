/*
This code is largely based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#include <limits>
#include <stdexcept>
#include "bitbots_splines/Combination.hpp"

namespace bitbots_splines {

unsigned long Combination::binomialCoefficient(
    size_t k, size_t n)
{
    if (n == 0 || k == 0) {
        return 1;
    }
    if (k > n) {
        throw std::logic_error("Combination not valid k>n");
    }

    if (n-k < k) {
        return binomialCoefficient(n-k, n);
    }
    if (k == 1 || k == n) {
        return n;
    }

    Pair pair(k, n);
    if (_pascalTriangle.count(pair) == 0) {
        unsigned long val1 = binomialCoefficient(k-1, n-1);
        unsigned long val2 = binomialCoefficient(k, n-1);
        unsigned long test = 
            std::numeric_limits<unsigned long>::max()
            - val1;
        if (val2 < test) {
            _pascalTriangle[pair] = val1 + val2;
        } else {
            throw std::runtime_error("Combination overflow");
        }
    }

    return _pascalTriangle[pair];
}
        
void Combination::startCombination(size_t k, size_t n)
{
    if (n == 0 || k == 0) {
        throw std::logic_error("Combination zero");
    }
    if (k > n) {
        throw std::logic_error("Combination not valid k>n");
    }
    
    _indexes = std::vector<size_t>();
    _k = k;
    _n = n;
    for (size_t i=0;i<k;i++) {
        _indexes.push_back(i);
    }
}
        
std::vector<size_t> Combination::nextCombination()
{
    std::vector<size_t> result = _indexes;
        
    if (_indexes.size() > 0) {
        bool isEnd = incrIndexes(_k-1);
        if (isEnd) {
            _indexes.clear();
        }
    }

    return result;
}

bool Combination::incrIndexes(size_t i)
{
    if (_indexes[i] == _n-(_k-i)) {
        if (i == 0) {
            return true;
        } else {
            bool isEnd = incrIndexes(i-1);
            if (isEnd) {
                return true;
            } else {
                _indexes[i] = _indexes[i-1] + 1;
                return false;
            }
        }
    } else {
        _indexes[i]++;
        return false;
    }
}

}

