/*
 * rei_graph.hpp
 *
 *  Created on: Sep 7, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_REI_CONSTRUCT_ELEMENTS_REI_GRAPH_HPP_
#define INCLUDE_REI_CONSTRUCT_ELEMENTS_REI_GRAPH_HPP_

#include <memory>
#include "rei_graph_exceptions.hpp"


namespace rei
{



enum class EdgeDirectionality {DIRECTED, UNDIRECTED};

namespace graph
{
// Edge forward definition
template<class CacheId, class Value> class Edge;
template<class CacheId, class Value> using EdgePtr = std::shared_ptr<Edge<CacheId, Value> >;
template<class CacheId, class Value> using WeakEdgePtr = std::weak_ptr<Edge<CacheId, Value> >;
// Graph forward definition
template<class CacheId, class Value> class Graph;

template<class CacheId, class Value> class Vertex
{
protected:
	std::string label;
	const unsigned int location_number;
	// Incoming edges
	std::map<CacheId, WeakEdgePtr<CacheId, Value> > incoming_edge_map;
	// Outgoing edges
	std::map<CacheId, WeakEdgePtr<CacheId, Value> > outgoing_edge_map;
public:
	Vertex(const std::string& label, unsigned int location_number):
		label(label), location_number(location_number){}

	virtual ~Vertex()
	{
		incoming_edge_map.erase(incoming_edge_map.begin(), incoming_edge_map.end());
		outgoing_edge_map.erase(outgoing_edge_map.begin(), outgoing_edge_map.end());
	}

	inline const std::string getLabel() const
	{
		return label;
	}

	inline const unsigned int getLocationNumber() const
	{
		return location_number;
	}

	WeakEdgePtr<CacheId, Value> getOutgoingEdge(CacheId cache_id)
	{
		auto it = outgoing_edge_map.find(cache_id);
		if (it!=outgoing_edge_map.end())	return it->second;
		throw exceptions::NonexistentEdge();
	}

	/*
	 * @brief: Add incoming edge to the node
	 */
	void addIncomingEdge(CacheId val, WeakEdgePtr<CacheId, Value> source)
	{
		incoming_edge_map.insert(std::pair<CacheId, WeakEdgePtr<CacheId, Value>>(
						val, source));
	}

	/*
	 * @brief: Add outgoing edge to the node
	 */
	void addOutgoingEdge(CacheId val, WeakEdgePtr<CacheId, Value> target)
	{
		outgoing_edge_map.insert(std::pair<CacheId, WeakEdgePtr<CacheId, Value>>(
						val, target));
	}

	friend class Graph<CacheId, Value>;

};

template<class CacheId, class Value> using VertexPtr = std::shared_ptr<Vertex<CacheId, Value> >;

template<class CacheId, class Value> class Edge
{
protected:
	std::string label;
	EdgeDirectionality directionality;
	const CacheId cache_id;
	Value value;
	// Graph edges
	// Edge source vertex
	VertexPtr<CacheId, Value> source_vertex;		///< Source vertex pointer
	// Edge target vertex
	VertexPtr<CacheId, Value> target_vertex;		///< Target vertex pointer
public:
	Edge(CacheId cache_id, Value value, VertexPtr<CacheId, Value> source, VertexPtr<CacheId, Value> target,
			EdgeDirectionality directionality = EdgeDirectionality::UNDIRECTED):
		cache_id(cache_id), value(value), source_vertex(source), target_vertex(target),
		directionality(directionality)
	{

	}

	virtual ~Edge()
	{
		source_vertex.reset();
		target_vertex.reset();
	}

	friend class Graph<CacheId, Value>;
};

template<class CacheId, class Value> class Graph
{
protected:
	unsigned long number_of_nodes;			///< Count the number of vertices
	// Name of the hybrid state machine
	std::string name;						///< Graph name
	// Store vertices in a vector
	std::vector<VertexPtr<CacheId, Value> > vertices;		///< Store locations in a vector of state pointers
	// Store edges in a vector
	std::vector<EdgePtr<CacheId, Value> >   edges;
	// Map labels to location
	std::map<std::string, VertexPtr<CacheId, Value>> label_to_node;
	// Map labels to edges
	std::map<std::string, EdgePtr<CacheId, Value>> label_to_edge;
public:
	Graph(const std::string name): number_of_nodes(0), name(name)
	{

	}

	virtual ~Graph()
	{
		label_to_node.erase(label_to_node.begin(), label_to_node.end());
		label_to_edge.erase(label_to_edge.begin(), label_to_edge.end());
		for (VertexPtr<CacheId, Value> v: vertices)
		{
			v.reset();
		}
		for (EdgePtr<CacheId, Value> e: edges)
		{
			e.reset();
		}
	}

	// Basic graph operations

	/*
	 * @brief: add a vertex to the graph
	 * @param: a shared pointer to the vertex
	 */
	void addVertex(VertexPtr<CacheId, Value> vertex)
	{
		label_to_node.insert(std::pair<std::string, VertexPtr<CacheId, Value>>(
			vertex->label, vertex));
		vertices.emplace_back(std::move(vertex));
		number_of_nodes++;
	}

	/*
	 * @brief: add an edge to the graph
	 */
	void addEdge(EdgePtr<CacheId, Value> edge, CacheId cache_id)
	{
		edge->source_vertex->addOutgoingEdge(cache_id, edge);
		edge->target_vertex->addIncomingEdge(cache_id, edge);
		label_to_edge.insert(std::pair<std::string, EdgePtr<CacheId, Value>>(edge->label, edge));
		edges.push_back(std::move(edge));
	}


	/*
	 * @brief: Get location labels in a list
	 */
	std::vector<std::string> getVertexLabels()
	{
		std::vector<std::string> labels;
		for (const auto& v: vertices)
		{
			labels.emplace_back(v->getLabel());
		}
		return labels;
	}

	/*
	 * @brief: Get name of the graph
	 */
	const std::string getName() const
	{
		return name;
	}
};

} // namespace graph

} // namespace rei


#endif /* INCLUDE_REI_CONSTRUCT_ELEMENTS_REI_GRAPH_HPP_ */
