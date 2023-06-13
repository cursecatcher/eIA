#!/usr/bin/env python3 

import argparse
import numpy as np 
import enum 
from typing import List, Set, Union
from itertools import chain 

import logging

class MapEncoding(enum.Enum):
    WALL = 0
    WALKABLE = 1
    DOOR = 2
    ROOM = 3 
    CORRIDOR = 4

    @classmethod
    def to_str(cls, evalue):
        if evalue is cls.DOOR:
            return "DOOR"
        elif evalue is cls.ROOM:
            return "ROOM"
        elif evalue is cls.CORRIDOR:
            return "CPOINT"


class Coordinates:
    def __init__(self, x, y, z = None) -> None:
        self.__point = np.array([x, y, z if z else 0])
    
    @property
    def vec(self):
        return self.__point

    @property
    def x(self):
        return self.__point[0]
    
    @property
    def y(self):
        return self.__point[1]
    
    def distance(self, other) -> Union[int, float]:
        assert isinstance(other, Coordinates)
        return np.sum( np.abs( self.__point - other.__point ) )   

    
    


class Vertex:
    """ A vertex is identified by a set of coordinates (x,y) and a label representing its type """

    def __init__(self, vid: int, 
                 coordinates: Coordinates,
                 typeof: MapEncoding) -> None:
        self.id = vid 
        self.coords = coordinates
        self.type = typeof


    def __str__(self):
        return f"{self.id} {MapEncoding.to_str( self.type )} {self.coords.x} {self.coords.y}"
    
    def __eq__(self, __value: object) -> bool:
        return isinstance(__value, Vertex) and self.id == __value.id


class GraphEdge:
    """ An edge connects two vertices and it is weighted as the distance between them  """

    def __init__(self, v1: Vertex, v2: Vertex): 
        self.v1, self.v2 = (v1, v2) if v1.id < v2.id else (v2, v1)
        self.w = v1.coords.distance( v2.coords ) #abs( v1.x - v2.x ) + abs( v1.y - v2.y )
        

    def __str__(self) -> str:
        return f"{self.v1.id} {self.v2.id} {self.w}"

    def __repr__(self) -> str:
        return f"{self.v1} <-> {self.v2}"
    
    def __hash__(self) -> int:
        return ( self.v1.id, self.v2.id ).__hash__()
    
    def __eq__(self, __value: object) -> bool:
        return isinstance( __value, GraphEdge ) and self.v1 == __value.v1 and self.v2 == __value.v2 


class SpatialGraph:
    """ A spatial graph is a representation of a map using an undirected labeled graph, where 
     vertices are identified by (x, y) coordinates and are labeled with the type of cell, 
     and edges are weighted as the Manhattan distance between the two vertices, which 
     are on the same horizontal/vertical line  """

    def __init__(self, mapsource: Union[ str, np.ndarray ] ) -> None:
        self.vertices = dict()
        self.edgelist = set() 

        if isinstance( mapsource, str ):
            with open( mapsource ) as f:
                # build a list of np.arrays using lines read from the input file 
                int_lines = [
                    np.array( [ int(char) for char in line.strip() ] )
                        for line in f 
                ]
                # build a matrix using each line as a row 
                self.matrix = np.vstack( int_lines ).T
                
                logging.info(f"Map shape: {self.matrix.shape}")
        elif isinstance( mapsource, np.ndarray ):
            self.matrix = mapsource
        else:
            raise RuntimeError("Please provide either a filename or a numpy matrix as input parameter.")

        self.__init_vertices()

        # ## now we have the coordinates of every graph vertex... we have to build edges 
        for vtype, vertices in self.vertices.items():
            logging.info(f"Counting {vtype}: {len(vertices)} vertices")

        self.__init_edges()


    def __init_vertices(self):
        # group vertices by their type, set  vertex id and identify coordinates
        id_count = 0

        ## create a vertex for each marker and assign them a unique id 
        for vtype in [ MapEncoding.DOOR, MapEncoding.ROOM, MapEncoding.CORRIDOR ]:
            curr = self.vertices[ vtype ] = [ 
                Vertex(i, coords, vtype) 
                    for i, coords in enumerate( self.__get_coordinates( vtype), start = id_count ) ]
            id_count += len( curr )


    def __init_edges(self):
        self.edgelist = self.__match_vertices( MapEncoding.ROOM, MapEncoding.DOOR )
        self.edgelist.update( self.__match_corridors( MapEncoding.DOOR) )
        self.edgelist.update( self.__match_corridors( MapEncoding.CORRIDOR) )

    def draw(self):
        self.__draw_with_graph_tool()

    def __draw_with_graph_tool(self):
        try:
            import graph_tool.all as gt 
        except ImportError:
            logging.critical("You have to install graph-tool module (https://graph-tool.skewed.de/) to use draw method!")
            return 

        graph = gt.Graph( None, directed=False )
        vertex_list = sorted( chain.from_iterable( self.vertices.values() ), key=lambda v: v.id )
        # print(vertex_list)


        graph.add_vertex( n = self.num_vertices )
        # gnames = graph.vertex_properties["name"] = graph.new_vertex_property("string")
        graph.vertex_properties["id"] = graph.new_vertex_property("int")
        graph.vertex_properties["id"].a = [ v.id for v in vertex_list ]

        graph.vertex_properties["label"] = graph.new_vertex_property("int")
        graph.vertex_properties["label"].a = [ v.type.value for v in vertex_list ]

        graph.vertex_properties["coords"] = graph.new_vertex_property("string")
        graph.vertex_properties["coords"].set_2d_array(
            np.array([ f"({v.coords.x}, {v.coords.y})" for v in vertex_list ])
        )

        colormap = {
            MapEncoding.DOOR.value: (1,0,0,1),  #the node is ignored by the feature set 
            MapEncoding.ROOM.value: (0,1,0,1),  #the node is positively correlated wrt the target
            MapEncoding.CORRIDOR.value: (0,0,1,1)   #the node is negatively correlated wrt the target 
        }
        # colorlist = [ colormap[v.type.value] for v in vertex_list]
        vprop_color = graph.new_vertex_property( "vector<double>" )
   
        for v_gt, v_obj in zip( graph.vertices(), vertex_list ): #self.__graph.vertices(), )
            vprop_color[ v_gt ] = colormap[ v_obj.type.value ]


        graph.add_edge_list( [ ( e.v1.id, e.v2.id ) for e in self.edgelist]  )

        args = dict( 
            vertex_text = graph.vp["coords"],
            # vertex_text_position = 1,
            # vertex_text = vprop,
            vertex_fill_color = vprop_color,
            vertex_text_position = .3,
            vertex_font_size = 7, 
            edge_pen_width=0.3,
            # vertex_size = vsize_p,
            output_size = ( 2000, 2000 ), 
            output = "graph_arf.pdf"
        )

        try:
            pos = gt.arf_layout(graph)
            gt.graph_draw( graph, pos=pos, **args )
            args["output"] = "graph_circle.pdf"
            state = gt.minimize_nested_blockmodel_dl(graph)
            gt.draw_hierarchy(state, **args) #output="celegansneural_nested_mdl.pdf")
        except Exception as e:
            logging.error(f"Error while drawing graph: {e}")

    @property
    def num_vertices(self):
        return sum( [ len( v_list ) for v_list in self.vertices.values() ] )
    
    @property
    def num_edges(self):
        return len( self.edgelist )
    

    def save(self, filename):
        with open(filename, "w") as f:
            ## GRAPH NAME 
            # NUMBER OF VERTICES
            f.write( f"{self.num_vertices}\n")
            # LIST OF VERTICES         
            vlist = sorted( chain.from_iterable( self.vertices.values() ), key = lambda v: v.id )
            f.writelines([ f"{v}\n" for v in vlist ])
            # NUMBER OF EDGES
            f.write( f"{self.num_edges}\n") 
            # LIST OF EDGES 
            sorted_edges = sorted( self.edgelist, key = lambda e: (e.v1.id, e.v2.id))
            f.writelines([ f"{e}\n"  for e in sorted_edges ])
            
    

    def __match_corridors(self, typeof: MapEncoding) -> Set[GraphEdge]:
        corridors = self.vertices.get( MapEncoding.CORRIDOR )
        points = self.vertices.get( typeof )

        edgelist = {
            GraphEdge( v_coo, v_point )
                for v_coo in corridors
                    for v_point in points 
                        if v_coo != v_point and self.__check_vertex_compatibility( v_coo, v_point )
        }
        logging.info(f"Corridors vs {typeof}: {len(edgelist)} edges")
        return edgelist


    def __match_vertices(self, type1: MapEncoding, type2: MapEncoding) -> List[GraphEdge]:
        """ Build edges between two vertices v1 and v2 s.t. type(v1) = type_v1 and type(v2) = type_v2
            and their coordinates are on the same line and without obstacles NEL MEZZO """
        
        points_t1 = self.vertices.get( type1 )
        points_t2 = self.vertices.get( type2 )
        bits_vector = [ False ] * len( points_t2 )

        edge_list = set()

        for v1 in points_t1:
            for i, (flag, v2)  in enumerate( zip( bits_vector, points_t2 ) ):
                if not flag and self.__check_vertex_compatibility( v1, v2 ):
                    bits_vector[ i ] = True 
                    edge_list.add( GraphEdge( v1, v2 ) )
                    break 


        try:
            assert all( bits_vector )
        except AssertionError:
            lv = [ v for v, bit in zip( points_t2, bits_vector) if not bit ]
            logging.warning(f"Unmatched vertices: {lv}")

        logging.info(f"{type1} vs {type2}: {len(edge_list)} edges")
        return edge_list
    


    def __get_coordinates(self, etype: MapEncoding ) -> List[ Coordinates ]:
        """ Given a vertex type, get (x, y) coordinates of those vertices in the map """

        return [ Coordinates(x, y, 0) for x, y in zip( *np.where( self.matrix == etype.value ) ) ]
    

    def __check_vertex_compatibility( self, v1: Vertex, v2: Vertex ) -> bool:
        """ Check if two vertices are on the same line (either horizontal or vertical) and there are no wall between them """

        matches = v1.coords.vec == v2.coords.vec
        if any( matches ):
            if matches[0]:# v1.x == v2.x:
                return self.__check_line( v1.coords.y, v2.coords.y, x = v1.coords.x )
            elif matches[1]: #v1.y == v2.y:
                return self.__check_line( v1.coords.x, v2.coords.x, y = v1.coords.y )

        return False

    def __check_line(self, lb, ub, x = None, y = None ) -> bool:
        """ Check if there are no obstacles between two aligned points """

        assert x or y 
        u, v = (lb, ub) if lb < ub else (ub, lb)
        data = self.matrix[ x, u:v ] if x else self.matrix[ u:v, y ]
        
        return np.all( data > 0 )




if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--map", type=str, required=True)
    parser.add_argument("-o", "--out_graph", type=str, required=True)
    args = parser.parse_args()

    logging.basicConfig(level = logging.INFO)


    graph = SpatialGraph( args.map )
    graph.save( args.out_graph)

    graph.draw()

