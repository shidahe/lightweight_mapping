//  Copyright 2011 David Lovi
//
//  This file is part of FreespaceDelaunayAlgorithm.
//
//  FreespaceDelaunayAlgorithm is free software: you can redistribute it
//  and/or modify it under the terms of the GNU General Public License as
//  published by the Free Software Foundation, either version 3 of the
//  License, or (at your option) any later version.
//
//  FreespaceDelaunayAlgorithm is distributed in the hope that it will be
//  useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with FreespaceDelaunayAlgorithm.  If not, see
//  <http://www.gnu.org/licenses/>.
//
//  As a special exception, you have permission to link this program
//  with the CGAL library and distribute executables, as long as you
//  follow the requirements of the GNU GPL in regard to all of the
//  software in the executable aside from CGAL.


// This header introduces the GraphWrapper_Boost class.  It thinly wraps part
// of the Boost Graph Library for computing graph cuts.

#ifndef __GRAPHWRAPPER_BOOST_H
#define __GRAPHWRAPPER_BOOST_H

#include <iostream>
#include <string>
#include <boost/graph/adjacency_list.hpp>
//#include <boost/graph/kolmogorov_max_flow.hpp>
#include <boost/graph/boykov_kolmogorov_max_flow.hpp>
#include <boost/graph/graph_utility.hpp>

namespace dlovi {
  using namespace std;
  using namespace boost;

  class GraphWrapper_Boost;
  class GraphWrapper_Boost {
  public:
    // Typedefs and Types
    typedef adjacency_list_traits<vecS, vecS, directedS> Traits;
    typedef adjacency_list<vecS, vecS, directedS,
                            property < vertex_name_t, std::string,
                            property < vertex_index_t, int,
                            property < vertex_color_t, boost::default_color_type,
                            property < vertex_distance_t, double,
                            property < vertex_predecessor_t, Traits::edge_descriptor > > > > >,

                            property < edge_capacity_t, double,
                            property < edge_residual_capacity_t, double,
                            property < edge_reverse_t, Traits::edge_descriptor > > > > Graph;


    // Constructors and Destructors
    GraphWrapper_Boost();
    GraphWrapper_Boost(int nVertices, int nEdges = 0);
    ~GraphWrapper_Boost();

    // Getters
    bool whatSegment(int node) const; // Returns true if source segment (vertex color is black), otherwise false
    int getSource() const;
    int getSink() const;

    // Public Methods
    void addNodes(int numVertices);
    void addSource();
    void addSink();
    void addTWeights(int node, double sourceWeight, double sinkWeight);
    void addEdge(int node1, int node2, double weight, double revWeight = 0.0);
    double maxflow();
    void print() const;
  private:
    // Private Members
    Graph m_g;
    Traits::vertex_descriptor m_s, m_t;
    property_map<Graph, edge_capacity_t>::type m_capacity;
    property_map<Graph, vertex_color_t>::type m_color;
    property_map<Graph, edge_reverse_t>::type m_rev;
  };
}

#endif
