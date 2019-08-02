# cpp_bagel_wrapper

Bagel (Biologically inspired Graph-Based Language) is a cross-platform
graph-based dataflow language developed at the
[Robotics Innovation Center of the German Research Center for Artificial Intelligence (DFKI-RIC)](http://robotik.dfki-bremen.de/en/startpage.html)
and the [University of Bremen](http://www.informatik.uni-bremen.de/robotik/index_en.php).
It runs on (Ubuntu) Linux, Mac and Windows.

This library provides a comfortable integration of `Bagel` graphs into C++ projects.

# General {#general}

The main user documentation of Bagel can be found at:
https://github.com/dfki-ric/bagel_wiki/wiki

The API documentation of `osg_graph_viz` can be build in the `doc`
sub-folder with the `make` command. The documentation is build into
the `doc/build` folder.

## General Usage

```
  // create new graph
  BagelGraph *graph = new BagelGraph();

  // load graph from file
  graph->loadGraph("test_graph.yml", "");

  // initialize interface
  std::vector<std::string> inputNames =  graph->getInputNames();
  std::vector<std::string> outputNames =  graph->getOutputNames();
  std::vector<double> inputs(inputNames.size());
  std::vector<double> outputs(outputNames.size());
  for(size_t i=0; i<inputs.size(); ++i) inputs[i] = 0.0;

  // set inputs; calculate graph; get outputs
  graph->setInputValues(inputs);
  graph->evaluate();
  graph->getOutputValues(&outputs);

```


# Test

A simple test and example is implemented in the test folder. It can be
executed with `cd build/test; ./test_graph; cd ../..`.


# License {#license}

cpp_bagel_wrapper is distributed under the
[3-clause BSD license](https://opensource.org/licenses/BSD-3-Clause).
