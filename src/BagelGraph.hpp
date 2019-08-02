/**
 * \author Malte Langosz (malte.langosz@dfki.de)
 * \version 0.1
 * \brief This library provides a confortable interface to c_bagel.
 *
 */

#ifndef _BAGEL_GRAPH_HPP_
#define _BAGEL_GRAPH_HPP_

#include <c_bagel/bagel.h>
#include <configmaps/ConfigData.h>
#include <string>
#include <map>
#include <vector>

namespace cpp_bagel_wrapper {

    /**
     * \author Malte Langosz (malte.langosz@dfki.de) \n
     * \brief "BagelGraph" can be used to load Bagel graphs. It uses the
     * c_bagel library and provides a reduced interface for general usage
     * of Bagel graphs in \c C++ applications.
     *
     */
  class BagelGraph {
  public:
    // todo: add copy contructor
    BagelGraph();
    ~BagelGraph();

    /**
     *\brief Load a Bagel graph file.
     *
     * \param name Is the filename of the graph to load. It have to be the full
     * path either absolute or relative to the current working directory.
     * \param externNodesPath If the graph makes use of extern nodes, this param
     * have to give the root path were the extern node libraries can be found.
     */
    void loadGraph(std::string name, std::string externNodesPath);

    /**
     *\brief Set an input value of the loaded graph.
     *
     * \param index The index of the input to set.
     * \param value The value to set.
     */
    void setInputValue(unsigned int index, double value);

    /**
     *\brief Set all input values of the loaded graph.
     *
     * \param v The vector of values to set. \warning If the length of the input vector
     * doesn't fit to the loaded graph's number of inputs none input is set.
     */
    void setInputValues(const std::vector<double> &v);

    /**
     *\brief Get the last outputs of the loaded graph.
     *
     * \param v A pointer to the vector of values to set. \warning If the length of the referenced vector
     * doesn't fit to the loaded graph's number of outputs none value is set.
     */
    void getOutputValues(std::vector<double> *v);

    /**
     *\brief Resets the loaded graph state.
     *
     */
    void reset();

    /**
     *\brief Evaluates the loaded graph and produces the next output values.
     *
     */
    void evaluate();

    /**
     *\brief Updates the content (graph) of a sub-graph node.
     *
     * \param filename Is used to identify the sub-graph that is updated.
     * \param graphMap Contains the Bagel graph that is replacing the original one.
     *
     */
    void updateSubGraph(const std::string &filename,
                        configmaps::ConfigMap graphMap);

    /**
     *\brief Returns a vector with the input names of the loaded Bagel graph.
     *
     * \return std::vector<std::string> The input names of the actual graph.
     */
    std::vector<std::string> getInputNames() {return inputNames;}

    /**
     *\brief Returns a vector with the output names of the loaded Bagel graph.
     *
     * \return std::vector<std::string> The output names of the actual graph.
     */
    std::vector<std::string> getOutputNames() {return outputNames;}

  private:
    bg_graph_t *graph;
    std::vector<double> inputs;
    std::vector<double> outputs;
    std::vector<bg_node_id_t> outputNodeIDs;
    std::vector<size_t> inputEdgeIDs;
    std::vector<bg_graph_t*> inputGraphs;
    std::vector<bg_graph_t*> outputGraphs;
    std::vector<std::string> inputNames;
    std::vector<std::string> outputNames;
    size_t edge_cnt;
    std::map<std::string, configmaps::ConfigMap> meta;
    void createSubInputs();
    void createSubOutputs();

  };

}  // end namespace cpp_bagel_wrapper

#endif // _BAGEL_GRAPH_HPP_
