#include "../src/BagelGraph.hpp"
#include <cmath>

using namespace cpp_bagel_wrapper;

int main(int argc, char** argv) {

  BagelGraph *graph = new BagelGraph();
  graph->loadGraph("test_graph.yml", "");
  std::vector<std::string> inputNames =  graph->getInputNames();
  std::vector<std::string> outputNames =  graph->getOutputNames();
  if(inputNames.size() != 4) {
    fprintf(stderr, "ERROR: wrong number of inputs loaded!");
    return -1;
  }
  if(outputNames.size() != 4) {
    fprintf(stderr, "ERROR: wrong number of outputs loaded!");
    return -1;
  }
  std::vector<double> inputs(inputNames.size());
  std::vector<double> outputs(outputNames.size());
  for(size_t i=0; i<4; ++i) inputs[i] = i;

  graph->setInputValues(inputs);
  graph->evaluate();
  graph->getOutputValues(&outputs);

  for(size_t i=0; i<4; ++i) {
    if(fabs(outputs[i] - i) > 0.000001) {
      fprintf(stderr, "ERROR: wrong output value at %lu with %g\n",
              i, outputs[i]);
      return -1;
    }
  }
  return 0;
}
