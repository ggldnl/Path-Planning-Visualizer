from model.geometry.polygon import Polygon
from model.geometry.point import Point

# Graphic
import matplotlib.pyplot as plt
import matplotlib.patches as patches


class QuadTreeNode:
    def __init__(self, bounds, max_polygons_per_region=4):
        self.bounds = bounds
        self.max_polygon_per_region = max_polygons_per_region
        self.children = [None, None, None, None]  # NW, NE, SW, SE
        self.polygons = []

    def insert(self, polygon_id, polygon):

        if not self.in_bounds(polygon.get_bounds()):
            return False

        if len(self.polygons) < self.max_polygon_per_region:  # If there's space, insert the polygon here
            self.polygons.append((polygon_id, polygon))
            return True

        # If the node is a leaf, split it into four children
        if all(child is None for child in self.children):
            self.split()

        # Recursively insert the polygon into one of the children
        """
        for i in range(len(self.children)):
            if self.children[i].insert(polygon_id, polygon):
                return True
        """
        inserted = []
        for i in range(len(self.children)):
            inserted.append(self.children[i].insert(polygon_id, polygon))

        return any(inserted)

    def remove(self, polygon_id):
        # Remove the polygon with the specified ID from the node
        self.polygons = [polygon for polygon in self.polygons if polygon[0] != polygon_id]

        # Recursively remove the polygon from the children
        for child in self.children:
            if child is not None:
                child.remove(polygon_id)

    def query_region(self, query_bounds):
        result = []

        # Check if the node's bounds intersect with the query region
        if not self.in_bounds(query_bounds):
            return result

        # Add IDs of polygons in the node that intersect with the query region
        for polygon_id, polygon in self.polygons:
            if self.intersects(polygon.get_bounds(), query_bounds):
                result.append(polygon_id)

        # Recursively query the children
        for child in self.children:
            if child is not None:
                result.extend(child.query_region(query_bounds))

        return result

    def in_bounds(self, polygon_bounds):
        min_x, min_y, max_x, max_y = self.bounds
        p_min_x, p_min_y, p_max_x, p_max_y = polygon_bounds
        return not (p_max_x < min_x or p_min_x > max_x or p_max_y < min_y or p_min_y > max_y)

    def split(self):
        min_x, min_y, max_x, max_y = self.bounds
        mid_x, mid_y = (min_x + max_x) / 2, (min_y + max_y) / 2

        self.children[0] = QuadTreeNode((mid_x, mid_y, max_x, max_y))  # NW
        self.children[1] = QuadTreeNode((min_x, mid_y, mid_x, max_y))  # NE
        self.children[2] = QuadTreeNode((min_x, min_y, mid_x, mid_y))  # SW
        self.children[3] = QuadTreeNode((mid_x, min_y, max_x, mid_y))  # SE

        # Reallocate polygons to children
        for polygon_id, polygon in self.polygons:
            for child in self.children:
                if child.in_bounds(polygon.get_bounds()):
                    child.insert(polygon_id, polygon)
                    # break

        self.polygons = []  # Clear polygons from the current node

    @staticmethod
    def intersects(bounds1, bounds2):

        """
                                    (max_x1, max_y1)
        +-------------------------------+
        |                               |   (max_x2, max_y2)
        |                         +-----------+
        |                         |     |     |
        +-------------------------|-----+     |
        (min_x1, min_y1)          |           |
                                  +-----------+
                            (min_x2, min_y2)
        """

        min_x1, min_y1, max_x1, max_y1 = bounds1
        min_x2, min_y2, max_x2, max_y2 = bounds2

        # Check for horizontal overlap
        horizontal_overlap = max_x1 > min_x2 and min_x1 < max_x2

        # Check for vertical overlap
        vertical_overlap = max_y1 > min_y2 and min_y1 < max_y2

        # Both horizontal and vertical overlap is required for rectangles to intersect
        return horizontal_overlap and vertical_overlap

        # return not (max_x1 < min_x2 or min_x1 > max_x2 or max_y1 < min_y2 or min_y1 > max_y2)

    def iterate(self):
        result = []

        # Add polygons in the node
        result.extend(self.polygons)

        # Recursively gather polygons from the children
        for child in self.children:
            if child is not None:
                result.extend(child.iterate())

        return result

    def draw(self, ax):
        min_x, min_y, max_x, max_y = self.bounds
        rect = patches.Rectangle((min_x, min_y), max_x - min_x, max_y - min_y, linewidth=1, edgecolor='b',
                                 facecolor='none')
        ax.add_patch(rect)

        for child in self.children:
            if child is not None:
                child.draw(ax)

        for polygon_id, polygon in self.polygons:
            p_min_x, p_min_y, p_max_x, p_max_y = polygon.get_bounds()
            rect = patches.Rectangle((p_min_x, p_min_y), p_max_x - p_min_x, p_max_y - p_min_y, linewidth=1,
                                     edgecolor='r', facecolor='none')
            ax.add_patch(rect)

            # Add text annotation in the middle of the polygon
            center_x = (p_min_x + p_max_x) / 2
            center_y = (p_min_y + p_max_y) / 2
            ax.text(center_x, center_y, str(polygon_id), ha='center', va='center', color='r')


class QuadTree:

    def __init__(self, bounds):
        self.root = QuadTreeNode(bounds)

    def insert(self, polygon_id, polygon):
        return self.root.insert(polygon_id, polygon)

    def remove(self, polygon_id):
        # Remove the polygon from the quad tree starting from the root
        self.root.remove(polygon_id)

    def query_region(self, query_bounds):
        # Query the quad tree starting from the root
        return self.root.query_region(query_bounds)

    def iterate(self):
        # Iterate over all polygons in the quad tree starting from the root
        return self.root.iterate()

    def __iter__(self):
        # Use the iterate_all_polygons method to make QuadTree iterable
        return iter(self.iterate())

    def draw(self):
        fig, ax = plt.subplots()
        ax.set_xlim(self.root.bounds[0], self.root.bounds[2])
        ax.set_ylim(self.root.bounds[1], self.root.bounds[3])
        self.root.draw(ax)

        return ax


# Example usage:
if __name__ == "__main__":

    quad_tree_bounds = (-5.0, -5.0, 5.0, 5.0)
    quad_tree = QuadTree(quad_tree_bounds)

    obstacles = [
            {
                "id": 0,
                "obstacle": {
                    "polygon": {
                        "points": [
                            {
                                "x": 3.23206078297003,
                                "y": -0.2400379661624355
                            },
                            {
                                "x": 3.826756158258299,
                                "y": -0.24171197872215716
                            },
                            {
                                "x": 3.825795312535151,
                                "y": -0.583053807135061
                            },
                            {
                                "x": 3.231099937246882,
                                "y": -0.5813797945753394
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": 3.5289280477525904,
                            "y": -0.41154588664874825
                        }
                    },
                    "vel": [
                        0,
                        0,
                        0
                    ]
                }
            },
            {
                "id": 1,
                "obstacle": {
                    "polygon": {
                        "points": [
                            {
                                "x": 0.6469100546890449,
                                "y": -0.7945465620359109
                            },
                            {
                                "x": 0.3198366237890558,
                                "y": -0.4926636984131564
                            },
                            {
                                "x": 0.7381368970272716,
                                "y": -0.03945842577006997
                            },
                            {
                                "x": 1.0652103279272607,
                                "y": -0.34134128939282443
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": 0.6925234758581582,
                            "y": -0.41700249390299043
                        }
                    },
                    "vel": [
                        0,
                        0,
                        0
                    ]
                }
            },
            {
                "id": 2,
                "obstacle": {
                    "polygon": {
                        "points": [
                            {
                                "x": 1.4442518886148101,
                                "y": -0.54755147856886
                            },
                            {
                                "x": 1.5511563156911536,
                                "y": -1.1503903342780657
                            },
                            {
                                "x": 1.2887934514655472,
                                "y": -1.196916452195847
                            },
                            {
                                "x": 1.1818890243892037,
                                "y": -0.5940775964866414
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": 1.3665226700401787,
                            "y": -0.8722339653823535
                        }
                    },
                    "vel": [
                        0,
                        0,
                        0
                    ]
                }
            },
            {
                "id": 3,
                "obstacle": {
                    "polygon": {
                        "points": [
                            {
                                "x": -0.7109717572163923,
                                "y": 0.10258116511210111
                            },
                            {
                                "x": -1.2121746269026539,
                                "y": -0.41681601874583973
                            },
                            {
                                "x": -1.665275790128083,
                                "y": 0.020413158316623464
                            },
                            {
                                "x": -1.1640729204418214,
                                "y": 0.5398103421745644
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": -1.1881237736722376,
                            "y": 0.061497161714362286
                        }
                    },
                    "vel": [
                        0,
                        0,
                        0
                    ]
                }
            },
            {
                "id": 4,
                "obstacle": {
                    "polygon": {
                        "points": [
                            {
                                "x": 4.380526652616933,
                                "y": -1.6511863751530902
                            },
                            {
                                "x": 3.718465910135744,
                                "y": -1.6581214374563682
                            },
                            {
                                "x": 3.7137537592507184,
                                "y": -1.2082725424942031
                            },
                            {
                                "x": 4.375814501731907,
                                "y": -1.2013374801909251
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": 4.047140205933825,
                            "y": -1.4297294588236464
                        }
                    },
                    "vel": [
                        0,
                        0,
                        0
                    ]
                }
            },
            {
                "id": 5,
                "obstacle": {
                    "polygon": {
                        "points": [
                            {
                                "x": -2.4287131126779937,
                                "y": -3.4402537408746703
                            },
                            {
                                "x": -2.7067897086060784,
                                "y": -2.942877997298162
                            },
                            {
                                "x": -2.501718857384843,
                                "y": -2.8282254333801466
                            },
                            {
                                "x": -2.2236422614567584,
                                "y": -3.325601176956655
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": -2.4652159850314184,
                            "y": -3.1342395871274085
                        }
                    },
                    "vel": [
                        0,
                        0,
                        0
                    ]
                }
            },
            {
                "id": 6,
                "obstacle": {
                    "polygon": {
                        "points": [
                            {
                                "x": -1.2759669912765523,
                                "y": -0.047864628442433194
                            },
                            {
                                "x": -1.538820539340585,
                                "y": -0.5341328682629288
                            },
                            {
                                "x": -2.13548226631643,
                                "y": -0.21160583670621222
                            },
                            {
                                "x": -1.8726287182523973,
                                "y": 0.27466240311428347
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": -1.7057246287964911,
                            "y": -0.1297352325743227
                        }
                    },
                    "vel": [
                        0,
                        0,
                        0
                    ]
                }
            },
            {
                "id": 7,
                "obstacle": {
                    "polygon": {
                        "points": [
                            {
                                "x": 1.9426289717921805,
                                "y": -1.1314758152519424
                            },
                            {
                                "x": 1.2551651682245462,
                                "y": -1.3962096733989457
                            },
                            {
                                "x": 1.1078771888460919,
                                "y": -1.0137306235636285
                            },
                            {
                                "x": 1.7953409924137262,
                                "y": -0.7489967654166252
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": 1.5252530803191362,
                            "y": -1.0726032194077855
                        }
                    },
                    "vel": [
                        0,
                        0,
                        0
                    ]
                }
            },
            {
                "id": 8,
                "obstacle": {
                    "polygon": {
                        "points": [
                            {
                                "x": 3.405874913450002,
                                "y": -2.3696939986730503
                            },
                            {
                                "x": 3.0411031336165575,
                                "y": -1.8333600392076692
                            },
                            {
                                "x": 3.429394347960041,
                                "y": -1.5692751815362533
                            },
                            {
                                "x": 3.7941661277934857,
                                "y": -2.1056091410016347
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": 3.4176346307050216,
                            "y": -1.9694845901046518
                        }
                    },
                    "vel": [
                        0,
                        0,
                        0
                    ]
                }
            },
            {
                "id": 9,
                "obstacle": {
                    "polygon": {
                        "points": [
                            {
                                "x": -2.7538064991600155,
                                "y": 1.105506429447202
                            },
                            {
                                "x": -2.908180809845169,
                                "y": 1.6930266051036682
                            },
                            {
                                "x": -2.459318264252424,
                                "y": 1.810967824723639
                            },
                            {
                                "x": -2.3049439535672707,
                                "y": 1.2234476490671726
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": -2.60656238170622,
                            "y": 1.4582371270854204
                        }
                    },
                    "vel": [
                        0,
                        0,
                        0
                    ]
                }
            },
            {
                "id": 10,
                "obstacle": {
                    "polygon": {
                        "points": [
                            {
                                "x": 2.565193662245759,
                                "y": -0.5876852583369581
                            },
                            {
                                "x": 3.1295113456466472,
                                "y": 0.01787008248489169
                            },
                            {
                                "x": 3.316334806702082,
                                "y": -0.1562309048908516
                            },
                            {
                                "x": 2.7520171233011936,
                                "y": -0.7617862457127014
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": 2.9407642344739204,
                            "y": -0.3719580816139048
                        }
                    },
                    "vel": [
                        0,
                        0,
                        0
                    ]
                }
            },
            {
                "id": 11,
                "obstacle": {
                    "polygon": {
                        "points": [
                            {
                                "x": -2.613531143002827,
                                "y": 3.4746609368842947
                            },
                            {
                                "x": -3.1835254120277203,
                                "y": 3.861977787057022
                            },
                            {
                                "x": -3.057712847646915,
                                "y": 4.047129661127061
                            },
                            {
                                "x": -2.4877185786220215,
                                "y": 3.6598128109543326
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": -2.8356219953248707,
                            "y": 3.760895299005677
                        }
                    },
                    "vel": [
                        0,
                        0,
                        0
                    ]
                }
            },
            {
                "id": 12,
                "obstacle": {
                    "polygon": {
                        "points": [
                            {
                                "x": -1.512652183089385,
                                "y": 0.665815095414746
                            },
                            {
                                "x": -1.3441984875240804,
                                "y": 1.5311994396345323
                            },
                            {
                                "x": -1.0837629977687602,
                                "y": 1.48050367537825
                            },
                            {
                                "x": -1.2522166933340648,
                                "y": 0.6151193311584637
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": -1.2982075904290726,
                            "y": 1.073159385396498
                        }
                    },
                    "vel": [
                        0,
                        0,
                        0
                    ]
                }
            },
            {
                "id": 13,
                "obstacle": {
                    "polygon": {
                        "points": [
                            {
                                "x": -0.5536761405009472,
                                "y": -0.9760429077209415
                            },
                            {
                                "x": 0.07841696935435222,
                                "y": -1.408953280513309
                            },
                            {
                                "x": -0.1341221459623128,
                                "y": -1.7192819739559102
                            },
                            {
                                "x": -0.7662152558176121,
                                "y": -1.2863716011635429
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": -0.34389914323162996,
                            "y": -1.347662440838426
                        }
                    },
                    "vel": [
                        0,
                        0,
                        0
                    ]
                }
            },
            {
                "id": 14,
                "obstacle": {
                    "polygon": {
                        "points": [
                            {
                                "x": 2.9703580470276267,
                                "y": -1.1799027861610198
                            },
                            {
                                "x": 2.641727662160103,
                                "y": -1.501771992108562
                            },
                            {
                                "x": 2.459240929713222,
                                "y": -1.3154519465462988
                            },
                            {
                                "x": 2.7878713145807454,
                                "y": -0.9935827405987567
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": 2.7147994883704243,
                            "y": -1.2476773663536593
                        }
                    },
                    "vel": [
                        0,
                        0,
                        0
                    ]
                }
            },
            {
                "id": 15,
                "obstacle": {
                    "polygon": {
                        "points": [
                            {
                                "x": -4.157123971373036,
                                "y": 1.4875536590437262
                            },
                            {
                                "x": -3.714132205377787,
                                "y": 1.2720915974157363
                            },
                            {
                                "x": -3.9708999560160483,
                                "y": 0.7441749960130213
                            },
                            {
                                "x": -4.4138917220112965,
                                "y": 0.9596370576410111
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": -4.064011963694542,
                            "y": 1.1158643275283737
                        }
                    },
                    "vel": [
                        0,
                        0,
                        0
                    ]
                }
            },
            {
                "id": 16,
                "obstacle": {
                    "polygon": {
                        "points": [
                            {
                                "x": 1.2311861967709785,
                                "y": -1.5675758920443164
                            },
                            {
                                "x": 1.728007575237695,
                                "y": -1.0025879459856113
                            },
                            {
                                "x": 2.2411589067105826,
                                "y": -1.4538268682367081
                            },
                            {
                                "x": 1.7443375282438662,
                                "y": -2.018814814295413
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": 1.7361725517407807,
                            "y": -1.5107013801405123
                        }
                    },
                    "vel": [
                        0,
                        0,
                        0
                    ]
                }
            },
            {
                "id": 17,
                "obstacle": {
                    "polygon": {
                        "points": [
                            {
                                "x": -2.905195779730727,
                                "y": 2.7021703370792958
                            },
                            {
                                "x": -3.1800690387954686,
                                "y": 2.2666554146996325
                            },
                            {
                                "x": -3.632660952204213,
                                "y": 2.5523067388500307
                            },
                            {
                                "x": -3.3577876931394712,
                                "y": 2.987821661229694
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": -3.26892836596747,
                            "y": 2.6272385379646632
                        }
                    },
                    "vel": [
                        0,
                        0,
                        0
                    ]
                }
            },
            {
                "id": 18,
                "obstacle": {
                    "polygon": {
                        "points": [
                            {
                                "x": -1.4904379632825486,
                                "y": 0.8852423502116309
                            },
                            {
                                "x": -1.1662088841707727,
                                "y": 0.253697207164719
                            },
                            {
                                "x": -1.7836931263856053,
                                "y": -0.06331314343895333
                            },
                            {
                                "x": -2.107922205497381,
                                "y": 0.5682319996079586
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": -1.637065544834077,
                            "y": 0.4109646033863388
                        }
                    },
                    "vel": [
                        0,
                        0,
                        0
                    ]
                }
            },
            {
                "id": 19,
                "obstacle": {
                    "polygon": {
                        "points": [
                            {
                                "x": 1.2409589874167843,
                                "y": -0.34761619525636944
                            },
                            {
                                "x": 0.8241460997161036,
                                "y": -0.9651991317166073
                            },
                            {
                                "x": 0.34627942792053246,
                                "y": -0.6426821454982722
                            },
                            {
                                "x": 0.7630923156212133,
                                "y": -0.025099209038034254
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": 0.7936192076686585,
                            "y": -0.4951491703773208
                        }
                    },
                    "vel": [
                        0,
                        0,
                        0
                    ]
                }
            },
            {
                "id": 20,
                "obstacle": {
                    "polygon": {
                        "points": [
                            {
                                "x": -3.9573237801517327,
                                "y": -1.5002933770318154
                            },
                            {
                                "x": -3.512433008522045,
                                "y": -2.248804579843597
                            },
                            {
                                "x": -4.0571070568866805,
                                "y": -2.5725411608771633
                            },
                            {
                                "x": -4.501997828516368,
                                "y": -1.824029958065382
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": -4.007215418519206,
                            "y": -2.0364172689544895
                        }
                    },
                    "vel": [
                        0,
                        0,
                        0
                    ]
                }
            },
            {
                "id": 21,
                "obstacle": {
                    "polygon": {
                        "points": [
                            {
                                "x": 2.585132376916565,
                                "y": 1.2315634906063924
                            },
                            {
                                "x": 2.8175752517143335,
                                "y": 0.8352192764398962
                            },
                            {
                                "x": 2.5790776172925702,
                                "y": 0.6953482406180536
                            },
                            {
                                "x": 2.346634742494801,
                                "y": 1.0916924547845497
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": 2.5821049971045675,
                            "y": 0.963455865612223
                        }
                    },
                    "vel": [
                        0,
                        0,
                        0
                    ]
                }
            },
            {
                "id": 22,
                "obstacle": {
                    "polygon": {
                        "points": [
                            {
                                "x": 1.5398828885800684,
                                "y": 3.61415902641988
                            },
                            {
                                "x": 1.271945413466987,
                                "y": 4.138560384074114
                            },
                            {
                                "x": 1.7664270939176296,
                                "y": 4.391210708135587
                            },
                            {
                                "x": 2.034364569030711,
                                "y": 3.866809350481353
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": 1.653154991248849,
                            "y": 4.0026848672777335
                        }
                    },
                    "vel": [
                        0,
                        0,
                        0
                    ]
                }
            },
            {
                "id": 23,
                "obstacle": {
                    "polygon": {
                        "points": [
                            {
                                "x": -1.4707468914154884,
                                "y": -0.15368245753458093
                            },
                            {
                                "x": -2.1057584688354987,
                                "y": 0.2762364349914911
                            },
                            {
                                "x": -1.8646726816046042,
                                "y": 0.6323321281663403
                            },
                            {
                                "x": -1.229661104184594,
                                "y": 0.2024132356402684
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": -1.6677097865100463,
                            "y": 0.23932483531587972
                        }
                    },
                    "vel": [
                        0,
                        0,
                        0
                    ]
                }
            },
            {
                "id": 24,
                "obstacle": {
                    "polygon": {
                        "points": [
                            {
                                "x": 2.773491033244116,
                                "y": 1.3507491976300958
                            },
                            {
                                "x": 2.617380297443915,
                                "y": 2.0495684176979037
                            },
                            {
                                "x": 3.047441749897078,
                                "y": 2.1456407749740127
                            },
                            {
                                "x": 3.2035524856972786,
                                "y": 1.446821554906205
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": 2.910466391570597,
                            "y": 1.7481949863020543
                        }
                    },
                    "vel": [
                        0,
                        0,
                        0
                    ]
                }
            },
            {
                "id": 25,
                "obstacle": {
                    "polygon": {
                        "points": [
                            {
                                "x": -3.3737697244202054,
                                "y": -1.7115784731721764
                            },
                            {
                                "x": -2.9963931987557926,
                                "y": -2.089739558120886
                            },
                            {
                                "x": -3.391940746130556,
                                "y": -2.4844664750473426
                            },
                            {
                                "x": -3.769317271794969,
                                "y": -2.1063053900986333
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": -3.3828552352753807,
                            "y": -2.0980224741097597
                        }
                    },
                    "vel": [
                        0,
                        0,
                        0
                    ]
                }
            },
            {
                "id": 26,
                "obstacle": {
                    "polygon": {
                        "points": [
                            {
                                "x": -0.30416247330399143,
                                "y": 1.1851888932791734
                            },
                            {
                                "x": -0.17002892356401816,
                                "y": 0.632580650374942
                            },
                            {
                                "x": -0.7367832070099127,
                                "y": 0.4950134590688445
                            },
                            {
                                "x": -0.8709167567498859,
                                "y": 1.0476217019730758
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": -0.520472840156952,
                            "y": 0.8401011761740089
                        }
                    },
                    "vel": [
                        0,
                        0,
                        0
                    ]
                }
            },
            {
                "id": 27,
                "obstacle": {
                    "polygon": {
                        "points": [
                            {
                                "x": -3.990260194614053,
                                "y": 1.7055206811589396
                            },
                            {
                                "x": -3.367754009365517,
                                "y": 2.270556525835288
                            },
                            {
                                "x": -2.915240865267321,
                                "y": 1.7720178335356138
                            },
                            {
                                "x": -3.537747050515857,
                                "y": 1.206981988859265
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": -3.452750529940687,
                            "y": 1.7387692573472768
                        }
                    },
                    "vel": [
                        0,
                        0,
                        0
                    ]
                }
            },
            {
                "id": 28,
                "obstacle": {
                    "polygon": {
                        "points": [
                            {
                                "x": 1.8297430240247752,
                                "y": -0.13848143511473288
                            },
                            {
                                "x": 1.8041030811535435,
                                "y": -0.8386407499504478
                            },
                            {
                                "x": 1.3648620637528333,
                                "y": -0.8225556756620362
                            },
                            {
                                "x": 1.390502006624065,
                                "y": -0.12239636082632122
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": 1.5973025438888042,
                            "y": -0.48051855538838456
                        }
                    },
                    "vel": [
                        0,
                        0,
                        0
                    ]
                }
            },
            {
                "id": 29,
                "obstacle": {
                    "polygon": {
                        "points": [
                            {
                                "x": 3.8627757255254838,
                                "y": 1.8123664756036932
                            },
                            {
                                "x": 3.327149033796254,
                                "y": 1.8959108900792283
                            },
                            {
                                "x": 3.4084147350021143,
                                "y": 2.4169281114124104
                            },
                            {
                                "x": 3.944041426731344,
                                "y": 2.3333836969368753
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": 3.635595230263799,
                            "y": 2.114647293508052
                        }
                    },
                    "vel": [
                        0,
                        0,
                        0
                    ]
                }
            },
            {
                "id": 30,
                "obstacle": {
                    "polygon": {
                        "points": [
                            {
                                "x": 0.04313812831988058,
                                "y": -3.934854775506788
                            },
                            {
                                "x": -0.3611860538047159,
                                "y": -3.305320797219355
                            },
                            {
                                "x": 0.1366279910095936,
                                "y": -2.9855949945434856
                            },
                            {
                                "x": 0.5409521731341901,
                                "y": -3.615128972830919
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": 0.0898830596647371,
                            "y": -3.460224885025137
                        }
                    },
                    "vel": [
                        0,
                        0,
                        0
                    ]
                }
            },
            {
                "id": 31,
                "obstacle": {
                    "polygon": {
                        "points": [
                            {
                                "x": -2.668239431533782,
                                "y": -0.7308155674845087
                            },
                            {
                                "x": -2.073029067775052,
                                "y": -0.9721977996339373
                            },
                            {
                                "x": -2.216486080059729,
                                "y": -1.3259400767906255
                            },
                            {
                                "x": -2.811696443818459,
                                "y": -1.0845578446411968
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": -2.4423627557967555,
                            "y": -1.028377822137567
                        }
                    },
                    "vel": [
                        0,
                        0,
                        0
                    ]
                }
            },
            {
                "id": 32,
                "obstacle": {
                    "polygon": {
                        "points": [
                            {
                                "x": -0.47890206287232084,
                                "y": -0.2892368082216574
                            },
                            {
                                "x": 0.09033476751876557,
                                "y": -0.8127288133214627
                            },
                            {
                                "x": -0.3824423104698594,
                                "y": -1.3268190434186642
                            },
                            {
                                "x": -0.9516791408609458,
                                "y": -0.803327038318859
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": -0.4306721866710901,
                            "y": -0.8080279258201609
                        }
                    },
                    "vel": [
                        0,
                        0,
                        0
                    ]
                }
            },
            {
                "id": 33,
                "obstacle": {
                    "polygon": {
                        "points": [
                            {
                                "x": 4.607596388202414,
                                "y": 0.3811382145214154
                            },
                            {
                                "x": 4.161261249202043,
                                "y": 0.28986934959354593
                            },
                            {
                                "x": 4.093366624498398,
                                "y": 0.6218966710511779
                            },
                            {
                                "x": 4.53970176349877,
                                "y": 0.7131655359790473
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": 4.350481506350406,
                            "y": 0.5015174427862966
                        }
                    },
                    "vel": [
                        0,
                        0,
                        0
                    ]
                }
            },
            {
                "id": 34,
                "obstacle": {
                    "polygon": {
                        "points": [
                            {
                                "x": -1.917106639735311,
                                "y": -1.5039404740377993
                            },
                            {
                                "x": -1.6147865473999525,
                                "y": -0.7361095597713346
                            },
                            {
                                "x": -1.3475790606846116,
                                "y": -0.8413178687304171
                            },
                            {
                                "x": -1.64989915301997,
                                "y": -1.6091487829968818
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": -1.6323428502099613,
                            "y": -1.1726291713841082
                        }
                    },
                    "vel": [
                        0,
                        0,
                        0
                    ]
                }
            },
            {
                "id": 35,
                "obstacle": {
                    "polygon": {
                        "points": [
                            {
                                "x": -2.7176699412219913,
                                "y": -0.8002994799962643
                            },
                            {
                                "x": -3.3906487036236985,
                                "y": -0.6941397556983219
                            },
                            {
                                "x": -3.3122865848565177,
                                "y": -0.19737846457903985
                            },
                            {
                                "x": -2.6393078224548105,
                                "y": -0.30353818887698225
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": -3.0149782630392545,
                            "y": -0.49883897228765206
                        }
                    },
                    "vel": [
                        0,
                        0,
                        0
                    ]
                }
            },
            {
                "id": 36,
                "obstacle": {
                    "polygon": {
                        "points": [
                            {
                                "x": 0.8275331227348508,
                                "y": 2.5019041569259084
                            },
                            {
                                "x": 1.2581229004568129,
                                "y": 3.0879268174767938
                            },
                            {
                                "x": 1.6760201986725642,
                                "y": 2.780869911895403
                            },
                            {
                                "x": 1.2454304209506022,
                                "y": 2.1948472513445174
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": 1.2517766607037075,
                            "y": 2.6413870344106556
                        }
                    },
                    "vel": [
                        0,
                        0,
                        0
                    ]
                }
            },
            {
                "id": 37,
                "obstacle": {
                    "polygon": {
                        "points": [
                            {
                                "x": -2.3429291789707887,
                                "y": 0.11179538440181874
                            },
                            {
                                "x": -2.9916719993822074,
                                "y": 0.3016537927855408
                            },
                            {
                                "x": -2.8229053410768272,
                                "y": 0.8783264765634462
                            },
                            {
                                "x": -2.1741625206654085,
                                "y": 0.6884680681797243
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": -2.582917260023808,
                            "y": 0.4950609304826325
                        }
                    },
                    "vel": [
                        0,
                        0,
                        0
                    ]
                }
            },
            {
                "id": 38,
                "obstacle": {
                    "polygon": {
                        "points": [
                            {
                                "x": 0.5723794995846352,
                                "y": 2.0238786496402765
                            },
                            {
                                "x": 0.4368620064287263,
                                "y": 1.498649167244738
                            },
                            {
                                "x": -0.12758530704914336,
                                "y": 1.6442854795709825
                            },
                            {
                                "x": 0.00793218610676552,
                                "y": 2.169514961966521
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": 0.22239709626774593,
                            "y": 1.8340820646056295
                        }
                    },
                    "vel": [
                        0,
                        0,
                        0
                    ]
                }
            },
            {
                "id": 39,
                "obstacle": {
                    "polygon": {
                        "points": [
                            {
                                "x": -3.4146006332864833,
                                "y": -2.9206513727423586
                            },
                            {
                                "x": -2.828606340581104,
                                "y": -2.5975124709578172
                            },
                            {
                                "x": -2.7232891706095743,
                                "y": -2.78849925771229
                            },
                            {
                                "x": -3.3092834633149537,
                                "y": -3.1116381594968314
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": -3.068944901948029,
                            "y": -2.8545753152273243
                        }
                    },
                    "vel": [
                        0,
                        0,
                        0
                    ]
                }
            }
        ]

    polygons = {}
    from model.world.map.obstacle import Obstacle
    for oid, obstacle_dict in enumerate(obstacles):
        obstacle = Obstacle.from_dict(obstacle_dict['obstacle'])
        polygons[oid] = obstacle.polygon

    # Insert polygons into the quad tree
    for polygon_id, polygon in polygons.items():
        quad_tree.insert(polygon_id, polygon)

    for polygon_with_id in quad_tree:
        pid = polygon_with_id[0]
        polygon = polygon_with_id[1]
        print(f'{pid}: {polygon}')

    print('-' * 100)

    # Query the polygon
    from model.geometry.segment import Segment
    start = Point(2.8, -0.46)
    end = Point(2.9, -0.6)
    segment = Segment(start, end)
    query_polygon = Polygon.segment_buffer(segment, 0.5, 0.5)
    query_bounds = query_polygon.get_bounds()

    result = quad_tree.query_region(query_bounds)
    print(f'Result: {result}')
    print(query_polygon)

    # Draw the quad tree
    ax = quad_tree.draw()

    # Draw the segment buffer
    x = [point.x for point in query_polygon.points] + [query_polygon.points[0].x]
    y = [point.y for point in query_polygon.points] + [query_polygon.points[0].y]
    ax.plot(x, y, color='g', linewidth=1)

    # Draw the query region
    p_min_x, p_min_y, p_max_x, p_max_y = query_bounds
    rect = patches.Rectangle((p_min_x, p_min_y), p_max_x - p_min_x, p_max_y - p_min_y, linewidth=1,
                             edgecolor='g', facecolor='none')
    ax.add_patch(rect)

    plt.show()


