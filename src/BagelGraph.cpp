#include "BagelGraph.hpp"
#include <mars/utils/misc.h>
#include <assert.h>

namespace cpp_bagel_wrapper {

  BagelGraph::BagelGraph() : graph(NULL), edge_cnt(0) {
    bg_initialize();
  }

  BagelGraph::~BagelGraph() { }

  void BagelGraph::loadGraph(std::string graphName,
                             std::string externNodesPath) {

    if(externNodesPath != ""){
      fprintf(stderr, "loading extern nodes:\n");
      getExternNodes(externNodesPath.c_str());
    }else{
      fprintf(stderr, "INFO: extern nodes are not loaded because 'externNodesPath' is not specified\n");
    }

    std::string filename = graphName;
    bg_error err;
    // check for suffix
    std::string suffix = mars::utils::getFilenameSuffix(graphName);
    if (suffix == "") {
      filename.append(".yml");
    }

    // clear old graph
    inputs.clear();
    outputs.clear();
    outputNodeIDs.clear();
    inputEdgeIDs.clear();
    inputNames.clear();
    inputGraphs.clear();
    outputNames.clear();
    outputGraphs.clear();
    edge_cnt = 0;
    if (graph != NULL) {
      err = bg_graph_free(graph);
      if (err != bg_SUCCESS) {
        fprintf(stderr, "ERROR (%d): Free old memory of Behavior Graph.\n",
                err);
        assert(0);
      }
    }

    // load graph
    err = bg_graph_alloc(&graph, graphName.c_str());
    if (err != bg_SUCCESS) {
      fprintf(stderr, "ERROR (%d): Allocating memory for Behavior Graph.\n",
              err);
      assert(0);
    }
    std::string prefix = mars::utils::getPathOfFile(filename);
    mars::utils::removeFilenamePrefix(&filename);
    fprintf(stderr, "BagelGraph::loadGraph: %s / %s\n",
            prefix.c_str(), filename.c_str());
    if (!prefix.empty()) {
      bg_graph_set_load_path(graph, prefix.c_str());
    }
    err = bg_graph_from_yaml_file(filename.c_str(), graph);
    if (err != bg_SUCCESS) {
      fprintf(stderr, "ERROR (%d): Allocating memory for Behavior Graph.\n",
              err);
      assert(0);
    }

    size_t inputCnt, outputCnt;
    bg_graph_get_input_nodes(graph, 0, &inputCnt);
    bg_graph_get_output_nodes(graph, 0, &outputCnt);

    fprintf(stderr, "[loadGraph] #inputs: %lu\n", inputCnt);
    fprintf(stderr, "[loadGraph] #outputs: %lu\n", outputCnt);

    bg_node_id_t inputIds[inputCnt];
    bg_graph_get_input_nodes(graph, inputIds, &inputCnt);

    bg_node_id_t outputIds[outputCnt];
    bg_graph_get_output_nodes(graph, outputIds, &outputCnt);

    /*
      for(unsigned int i=0; i<inputCnt; ++i) {
      printf("inputNode: %lu\n", inputIds[i]);
      }
    */

    const char *name;
    for (unsigned int i = 0; i < outputCnt; ++i) {
      outputNodeIDs.push_back(outputIds[i]);
      outputs.push_back(0.0);
      outputGraphs.push_back(graph);
      //printf("outputNode: %lu\n", outputIds[i]);
      err = bg_node_get_name(graph, outputIds[i], &name);
      outputNames.push_back(name);
      //printf("outputNode: %lu %s\n", outputIds[i], name);
    }

    // create connection to give input to the network
    err =  bg_graph_get_edge_cnt(graph, false, &edge_cnt);
    if (err != bg_SUCCESS) {
      fprintf(stderr, "behavior_graph error: %d\n", err);
      assert(false);
    }

    for (size_t i = 0; i < inputCnt; ++i) {
      size_t edgeId = i + edge_cnt + 1;
      err = bg_graph_create_edge(graph, 0, 0, inputIds[i], 0, 1., edgeId);
      if (err != bg_SUCCESS) {
        fprintf(stderr, "behavior_graph error: %d\n", err);
        assert(false);
      }
      err = bg_node_get_name(graph, inputIds[i], &name);
      if (err != bg_SUCCESS) {
        fprintf(stderr, "behavior_graph error: %d\n", err);
        assert(false);
      }
      inputNames.push_back(name);
      inputs.push_back(0.0);
      inputGraphs.push_back(graph);
      //fprintf(stderr, "read inputName: %lu %lu %s\n", inputIds[i], edgeId, name);
      inputEdgeIDs.push_back(edgeId);
    }

    filename = graphName;
    mars::utils::removeFilenameSuffix(&filename);
    filename += "_meta.yml";

    if(mars::utils::pathExists(filename)){
      configmaps::ConfigMap map, debugMap;
      map = configmaps::ConfigMap::fromYamlFile(filename);
      meta[graphName] = map;
      if (!meta[graphName].empty()) {
        debugMap[meta[graphName].begin()->first] = meta[graphName].begin()->second;
      }
    }

    char **pathList, **fileList;
    size_t subCount;
    bg_graph_get_subgraph_list(graph, 0, 0, &subCount, true);
    pathList = (char**)calloc(subCount, sizeof(char*));
    fileList = (char**)calloc(subCount, sizeof(char*));
    for (size_t i = 0; i < subCount; ++i) {
      pathList[i] = (char*)malloc(sizeof(char) * 255);
      fileList[i] = (char*)malloc(sizeof(char) * 255);
    }
    // ToDo: memory leak?
    bg_graph_get_subgraph_list(graph, pathList, fileList, &subCount, true);
    for (size_t i = 0; i < subCount; ++i) {
      //fprintf(stderr, "subgraph: %s - %s\n", pathList[i], fileList[i]);

      filename = fileList[i];
      if (meta.find(filename) == meta.end()) {
        mars::utils::removeFilenameSuffix(&filename);
        filename += "_meta.yml";
        if(mars::utils::pathExists(filename)){
          configmaps::ConfigMap map, debugMap;
          map = configmaps::ConfigMap::fromYamlFile(filename);
          meta[fileList[i]] = map;
          if (!meta[fileList[i]].empty()) {
            debugMap[meta[fileList[i]].begin()->first] = meta[fileList[i]].begin()->second;
          }
        }
      }
    }
    //debugMap.toYamlFile("da.yml");
    //createSubInputs();
    //createSubOutputs();
  }

  void BagelGraph::createSubInputs() {
    bg_error err;
    const char *name;
    std::string input;
    configmaps::ConfigMap map;
    configmaps::ConfigVector::iterator it;
    map = configmaps::ConfigMap::fromYamlFile("SubInputs.yml");
    std::vector<std::string> arrName;
    bg_graph_t *tGraph;

    for (it = map["subInputs"].begin(); it != map["subInputs"].end(); ++it) {
      input = (*it)["name"].getString();

      tGraph = graph;
      arrName = mars::utils::explodeString('/', input);
      for (size_t i = 0; i < arrName.size() - 1; ++i) {
        //fprintf(stderr, "da: %lu %s\n", i, arrName[i].c_str());
        err = bg_graph_get_subgraph(tGraph, arrName[i].c_str(), &tGraph);
        if (tGraph == NULL) {
          fprintf(stderr, "did not found subgraph: %s\n", arrName[i].c_str());
          assert(false);
        }
      }

      size_t subInputCnt;
      bg_graph_get_input_nodes(tGraph, 0, &subInputCnt);
      bg_node_id_t subInputIds[subInputCnt];
      bg_graph_get_input_nodes(tGraph, subInputIds, &subInputCnt);
      for (size_t i = 0; i < subInputCnt; ++i) {
        err = bg_node_get_name(tGraph, subInputIds[i], &name);
        if (err != bg_SUCCESS) {
          fprintf(stderr, "behavior_graph error: %d\n", err);
          assert(false);
        }
        if (arrName.back().compare(name) == 0) {
          size_t edgeId = inputNames.size() + edge_cnt + 1;
          err = bg_graph_create_edge(tGraph, 0, 0, subInputIds[i], 0, 1., edgeId);
          if (err != bg_SUCCESS) {
            fprintf(stderr, "behavior_graph error: %d\n", err);
            assert(false);
          }
          //fprintf(stderr, "add: %s\n", input.c_str());
          inputNames.push_back(input);
          inputs.push_back(0.0);
          inputGraphs.push_back(tGraph);
          inputEdgeIDs.push_back(edgeId);
          if (err != bg_SUCCESS) {
            fprintf(stderr, "behavior_graph error: %d\n", err);
            assert(false);
          }
        }
      }
    }
  }


  void BagelGraph::createSubOutputs() {
    bg_error err;
    const char *name;
    std::string output;
    configmaps::ConfigMap map;
    configmaps::ConfigVector::iterator it;
    map = configmaps::ConfigMap::fromYamlFile("SubOutputs.yml");
    std::vector<std::string> arrName;
    bg_graph_t *tGraph;

    for (it = map["subOutputs"].begin(); it != map["subOutputs"].end(); ++it) {
      output = (*it)["name"].getString();

      tGraph = graph;
      arrName = mars::utils::explodeString('/', output);
      for (size_t i = 0; i < arrName.size() - 1; ++i) {
        //fprintf(stderr, "da: %lu %s\n", i, arrName[i].c_str());
        err = bg_graph_get_subgraph(tGraph, arrName[i].c_str(), &tGraph);
        if (tGraph == NULL) {
          fprintf(stderr, "did not found subgraph: %s\n", arrName[i].c_str());
          assert(false);
        }
      }
      size_t subOutputCnt;
      bg_graph_get_output_nodes(tGraph, 0, &subOutputCnt);
      bg_node_id_t subOutputIds[subOutputCnt];
      bg_graph_get_output_nodes(tGraph, subOutputIds, &subOutputCnt);
      for (size_t i = 0; i < subOutputCnt; ++i) {
        err = bg_node_get_name(tGraph, subOutputIds[i], &name);
        if (err != bg_SUCCESS) {
          fprintf(stderr, "behavior_graph error: %d\n", err);
          assert(false);
        }
        if (arrName.back().compare(name) == 0) {
          //fprintf(stderr, "add: %s\n", output.c_str());
          outputNames.push_back(output);
          outputs.push_back(0.0);
          outputGraphs.push_back(tGraph);
          outputNodeIDs.push_back(subOutputIds[i]);
        }
      }
    }
  }

  void BagelGraph::setInputValue(unsigned int index, double value) {
    if (index < inputs.size()) {
      inputs[index] = value;
    }
    else {
      assert(0);
    }
  }

  void BagelGraph::setInputValues(const std::vector<double> &v) {
    if (v.size() == inputs.size()) {
      inputs = v;
    }
    else {
      fprintf(stderr, "BAGEL Graph has %lu inputs, but %lu shall be set.\n", inputs.size(), v.size());
      assert(0);
    }
  }

  void BagelGraph::getOutputValues(std::vector<double>* v) {
    if (v->size() == outputs.size()) {
      *v = outputs;
    }
    else {
      assert(0);
    }
  }

  void BagelGraph::reset() {
    if (graph) {
      bg_graph_reset(graph, true);
    }
    else {
      assert(0);
    }
  }

  void BagelGraph::evaluate() {
    bg_error err;

    if (graph) {
      for (unsigned int k = 0; k < inputs.size(); k++) {
        err = bg_edge_set_value(inputGraphs[k], inputEdgeIDs[k], inputs[k]);
        if (err != bg_SUCCESS) {
          fprintf(stderr, "ERROR (%d): set value of edge %lu\n", err,
                  inputEdgeIDs[k]);
          assert(false);
        }
      }

      err = bg_graph_evaluate(graph);
      if (err != bg_SUCCESS) {
        fprintf(stderr, "ERROR (%d): bg_graph_evaluate\n", err);
        assert(false);
      }

      bg_real out;
      for (unsigned int k = 0; k < outputNames.size(); ++k) {

        err = bg_node_get_output(outputGraphs[k], outputNodeIDs[k], 0,
                                 &out);
        outputs[k] = out;
        if (err != bg_SUCCESS) {
          fprintf(stderr, "ERROR (%d): while reading output %lu\n", err,
                  outputNodeIDs[k]);
          assert(false);
        }
      }
    }
    else {
      assert(0);
    }
  }

  void BagelGraph::updateSubGraph(const std::string &filename,
                                  configmaps::ConfigMap graphMap) {
    
    std::string graphString = graphMap.toYamlString();
    bg_error err;
    bg_graph_t *subGraph;

    std::string file = filename;
    std::string prefix = mars::utils::getPathOfFile(file);
    mars::utils::removeFilenamePrefix(&file);

    err = bg_graph_alloc(&subGraph, file.c_str());
    // todo: use abort here
    assert(err == bg_SUCCESS);

    if (!prefix.empty()) {
      bg_graph_set_load_path(subGraph, prefix.c_str());
    }
    err = bg_graph_from_yaml_string((const unsigned char*)graphString.c_str(), subGraph);
    assert(err == bg_SUCCESS);
    err = bg_graph_set_subgraph(graph, file.c_str(), subGraph);
    assert(err == bg_SUCCESS);
    bg_graph_free(subGraph);
    assert(err == bg_SUCCESS);
  }

} // cpp_bagel_wrapper
