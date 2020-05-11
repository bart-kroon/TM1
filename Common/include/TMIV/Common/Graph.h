#ifndef _TMIV_COMMON_GRAPH_H_
#define _TMIV_COMMON_GRAPH_H_

#include "Matrix.h"
#include <functional>
#include <queue>

namespace TMIV::Common {
namespace Graph {
using NodeId = std::size_t;
enum class LinkType { Undirected, Directed };
has_type_helper(weight_type);

template <typename T> class Link {
private:
  NodeId m_node;
  T m_weight;

public:
  Link(NodeId n, T w) : m_node(n), m_weight(w) {}
  [[nodiscard]] auto node() const -> NodeId { return m_node; }
  [[nodiscard]] auto weight() const -> T { return m_weight; }
};

template <typename T> class Edge {
private:
  std::pair<NodeId, NodeId> m_nodes;
  T m_weight;

public:
  Edge(NodeId i, NodeId j, T w) : m_nodes({i, j}), m_weight(w) {}
  [[nodiscard]] auto nodes() const -> const std::pair<NodeId, NodeId> & { return m_nodes; }
  auto weight() const -> T { return m_weight; }
};

template <typename T> class Base {
public:
  using weight_type = T;

public:
  //! \brief  Return the number of nodes of the graph.
  [[nodiscard]] virtual auto getNumberOfNodes() const -> std::size_t = 0;
  //! \brief  Return the number of neighbours of node.
  [[nodiscard]] virtual auto getNeighbourhoodSize(NodeId node) const -> std::size_t = 0;
  //! \brief  Return the #id neighbour of node.
  [[nodiscard]] virtual auto getNeighbour(NodeId node, std::size_t id) const -> Link<T> = 0;
};

namespace BuiltIn {
//! \brief Regular built-in graph
template <typename T> class Sparse : public Base<T> {
public:
  using weight_type = T;

private:
  std::vector<std::vector<Link<T>>> m_link;

public:
  explicit Sparse(std::size_t nb_nodes = 0) : m_link{nb_nodes} {}
  void addNode() { m_link.emplace_back({}); }
  [[nodiscard]] auto getNumberOfNodes() const -> std::size_t override { return m_link.size(); }
  [[nodiscard]] auto getNeighbourhoodSize(NodeId node) const -> std::size_t override {
    return m_link[node].size();
  }
  [[nodiscard]] auto getNeighbour(NodeId node, std::size_t id) const -> Link<T> override {
    return m_link[node][id];
  }
  [[nodiscard]] auto getNeighbourhood(NodeId id) const -> const std::vector<Link<T>> & {
    return m_link[id];
  }
  //! \brief Connect vertices node and other and specify the connection weight.
  void connect(NodeId node, NodeId other, T weight, LinkType type) {
    m_link[node].push_back(Link<T>(other, weight));
    if (type == LinkType::Undirected) {
      m_link[other].push_back(Link<T>(node, weight));
    }
  }
  //! \brief Add the specified edge.
  void addEdge(const Edge<T> &e, LinkType type) {
    connect(e.nodes().first, e.nodes().second, e.weight(), type);
  }
};

//! \brief Dense built-in graph
template <typename T> class Dense : public Base<T> {
public:
  using weight_type = T;

private:
  Mat<T> m_weight;

public:
  Dense(Mat<T> weight) : m_weight(std::move(weight)) {}
  auto getWeightMatrix() const -> const Mat<T> & { return m_weight; }
  [[nodiscard]] auto getNumberOfNodes() const -> std::size_t override { return m_weight.m(); }
  [[nodiscard]] auto getNeighbourhoodSize(NodeId /*unused*/) const -> std::size_t override {
    return m_weight.m();
  }
  auto getNeighbour(NodeId node, NodeId id) const -> Link<T> override {
    return Link<T>(id, m_weight(node, id));
  }
  //! \brief Connect vertices node and other and specify the connection weight.
  void connect(NodeId node, NodeId other, T weight, LinkType type) {
    m_weight(node, other) = weight;
    if (type == LinkType::Undirected) {
      m_weight(other, node) = weight;
    }
  }
};

} // namespace BuiltIn
auto getDescendingOrderId(const BuiltIn::Sparse<float> &g) -> std::vector<NodeId>;
auto getReversedGraph(const BuiltIn::Sparse<float> &g) -> BuiltIn::Sparse<float>;

} // namespace Graph

} // namespace TMIV::Common

//! \brief Send the graph g to the stream os.
template <typename G, std::enable_if_t<TMIV::Common::Graph::has_weight_type<G>::value, int> = 0>
auto operator<<(std::ostream &os, const G &g) -> std::ostream & {
  for (std::size_t i = 0; i < g.getNumberOfNodes(); i++) {
    os << "n" << i << " -> ";
    for (std::size_t j = 0; j < g.getNeighbourhoodSize(i); j++) {
      auto l = g.getNeighbour(i, j);
      os << "n" << l.node() << "[" << l.weight() << "] ";
    }
    if (i != (g.getNumberOfNodes() - 1)) {
      os << '\n';
    }
  }

  return os;
}

#endif
