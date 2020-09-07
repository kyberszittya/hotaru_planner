/*
 * rei_graph.hpp
 *
 *  Created on: Sep 7, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_REI_CONSTRUCT_ELEMENTS_REI_GRAPH_HPP_
#define INCLUDE_REI_CONSTRUCT_ELEMENTS_REI_GRAPH_HPP_

#include <memory>

namespace rei
{

namespace graph
{

template<class CacheId, class Value> class Edge;
template<class CacheId, class Value> using EdgePtr = std::shared_ptr<Edge<CacheId, Value> >;
template<class CacheId, class Value> using WeakEdgePtr = std::weak_ptr<Edge<CacheId, Value> >;

template<class CacheId, class Value> class Vertex
{
protected:
	std::string label;
	const unsigned int location_number;
	// Event transition map
	std::map<CacheId, WeakEdgePtr<CacheId, Value> > outgoing_edge_map;
public:
	Vertex(const std::string& label, unsigned int location_number):
		label(label), location_number(location_number){}

	virtual ~Vertex()
	{
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
		return outgoing_edge_map[cache_id];
	}

	/*
	 * @brief: Add outgoing edge to the node
	 */
	void addOutgoingEdge(CacheId val, WeakEdgePtr<CacheId, Value> target)
	{
		outgoing_edge_map.insert(std::pair<CacheId, WeakEdgePtr<CacheId, Value>>(
						val, target));
	}


};

template<class CacheId, class Value> using VertexPtr = std::shared_ptr<Vertex<CacheId, Value> >;

template<class CacheId, class Value> class Edge
{
protected:
	std::string label;
	const CacheId cache_id;
	Value value;
	// Graph edges
	// Edge source vertex
	VertexPtr<CacheId, Value> source_vertex;		///< Source vertex pointer
	// Edge target vertex
	VertexPtr<CacheId, Value> target_vertex;		///< Target vertex pointer
public:
	Edge(CacheId cache_id, Value value, VertexPtr<CacheId, Value> source, VertexPtr<CacheId, Value> target):
		cache_id(cache_id), value(value), source_vertex(source), target_vertex(target)
	{

	}

	virtual ~Edge()
	{
		source_vertex.reset();
		target_vertex.reset();
	}
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
public:
	Graph(const std::string name): number_of_nodes(0), name(name)
	{

	}

	virtual ~Graph()
	{
		label_to_node.erase(label_to_node.begin(), label_to_node.end());
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
		vertices.emplace_back(std::move(vertex));
		label_to_node.insert(std::pair<std::string, VertexPtr<CacheId, Value>>(
			vertices.back()->getLabel(), vertices.back()));
		number_of_nodes++;
	}

	/*
	 * @brief: add an edge to the graph
	 */
	void addEdge(EdgePtr<CacheId, Value> edge)
	{
		edges.push_back(std::move(edge));
	}

	/*
	 * @brief: Get location labels in a list
	 */
	std::vector<std::string> getLocationLabels()
	{
		std::vector<std::string> labels;
		for (const auto& v: vertices)
		{
			labels.emplace_back(v->getLabel());
		}
		return labels;
	}
};

} // namespace graph

} // namespace rei


#endif /* INCLUDE_REI_CONSTRUCT_ELEMENTS_REI_GRAPH_HPP_ */
