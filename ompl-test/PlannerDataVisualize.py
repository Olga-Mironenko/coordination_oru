#!/usr/bin/env python3

# Based on `PlannerData.py` from OMPL demos.

import sys

import graph_tool.all as gt


def main(filename_in="cmake-build-debug/pd.graphml"):
    filename_out = filename_in + '.png'

    # Load the graphml data using graph-tool
    graph = gt.load_graph(filename_in, fmt="xml")

    edgeweights = graph.edge_properties["weight"]

    coordstrings = graph.vertex_properties["coords"]
    pos = graph.new_vertex_property("vector<double>")
    for i_vertex, coordstring in enumerate(coordstrings):
        x, y = [float(value) for value in coordstring.split(",")][:2]
        pos[i_vertex] = (x, -y)

    # Write some interesting statistics
    avgdeg, stddevdeg = gt.vertex_average(graph, "total")
    avgwt, stddevwt = gt.edge_average(graph, edgeweights)

    print("---- PLANNER DATA STATISTICS ----")
    print(str(graph.num_vertices()) + " vertices and " + str(graph.num_edges()) + " edges")
    print("Average vertex degree (in+out) = " + str(avgdeg) + "  St. Dev = " + str(stddevdeg))
    print("Average edge weight = " + str(avgwt)  + "  St. Dev = " + str(stddevwt))

    _, hist = gt.label_components(graph)
    print("Strongly connected components: " + str(len(hist)))

    # Make the graph undirected (for weak components, and a simpler drawing)
    graph.set_directed(False)
    _, hist = gt.label_components(graph)
    print("Weakly connected components: " + str(len(hist)))

    # Plotting the graph
    gt.remove_parallel_edges(graph) # Removing any superfluous edges

    edgeweights = graph.edge_properties["weight"]
    colorprops = graph.new_vertex_property("string")
    vertexsize = graph.new_vertex_property("double")

    start = -1
    goal = -1

    for v in range(graph.num_vertices()):

        # Color and size vertices by type: start, goal, other
        if v == 0:
            start = v
            colorprops[graph.vertex(v)] = "cyan"
            vertexsize[graph.vertex(v)] = 10
        elif v == 1:
            goal = v
            colorprops[graph.vertex(v)] = "green"
            vertexsize[graph.vertex(v)] = 10
        else:
            colorprops[graph.vertex(v)] = "yellow"
            vertexsize[graph.vertex(v)] = 5

    # default edge color is black with size 0.5:
    edgecolor = graph.new_edge_property("string")
    edgesize = graph.new_edge_property("double")
    for e in graph.edges():
        edgecolor[e] = "black"
        edgesize[e] = 0.5

    """
    # using A* to find shortest path in planner data
    if start != -1 and goal != -1:
        _, pred = gt.astar_search(graph, graph.vertex(start), edgeweights)

        # Color edges along shortest path red with size 3.0
        v = graph.vertex(goal)
        while v != graph.vertex(start):
            p = graph.vertex(pred[v])
            for e in p.out_edges():
                if e.target() == v:
                    edgecolor[e] = "red"
                    edgesize[e] = 2.0
            v = p
    """

    # pos indicates the desired vertex positions
    gt.graph_draw(
        graph,
        vertex_size=vertexsize, vertex_fill_color=colorprops, pos=pos,
        edge_pen_width=edgesize, edge_color=edgecolor,
        output_size=(1859, 1658),
        # output=filename_out,
    )
    print('\nDone')


if __name__ == "__main__":
    main(*sys.argv[1:])