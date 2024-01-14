#include "Common.h"
#include "esp_random.h"
#include <cstdio>
// #include <random>
// using namespace std;
int random_int(int min, int max) {
  // // Create a random device object to seed the generator
  // random_device rd;
  // // Create a default random engine object with the seed
  // default_random_engine gen(rd());
  // // Create a uniform distribution object with the range
  // uniform_int_distribution<int> dist(min, max);
  // // Return a random integer from the distribution
  // return dist(gen);
  int ret = (esp_random() % (max - min)) - min;
  return ret;
}
