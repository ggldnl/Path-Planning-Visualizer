from model.geometry.polygon import Polygon
from model.geometry.point import Point

"""
# Graphic
import matplotlib.pyplot as plt
import matplotlib.patches as patches
"""


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
        for i in range(len(self.children)):
            if self.children[i].insert(polygon_id, polygon):
                return True

    def remove(self, polygon_id):
        # Remove the polygon with the specified ID from the node
        self.polygons = [polygon for polygon in self.polygons if polygon[0] != polygon_id]

        """
        for polygon in self.polygons:
            print(f'{polygon}')
        """

        # Recursively remove the polygon from the children
        for child in self.children:
            if child is not None:
                child.remove(polygon_id)

    def query_region(self, query_bounds):
        result = []

        # Check if the node's bounds intersect with the query region
        if not self.in_bounds(query_bounds):
            return result

        for polygon_id, polygon in self.polygons:
            if polygon_id == 32:
                print('-')

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
                    break

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

    """
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
    """


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

    """
    def draw(self):
        fig, ax = plt.subplots()
        ax.set_xlim(self.root.bounds[0], self.root.bounds[2])
        ax.set_ylim(self.root.bounds[1], self.root.bounds[3])
        self.root.draw(ax)
        plt.show()
    """


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
                                "x": 0.927773661422274,
                                "y": -2.2676846827435875
                            },
                            {
                                "x": 0.4202647842998554,
                                "y": -1.644643967487323
                            },
                            {
                                "x": 0.8220206789431359,
                                "y": -1.3173865700572698
                            },
                            {
                                "x": 1.3295295560655545,
                                "y": -1.9404272853135343
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": 0.874897170182705,
                            "y": -1.7925356264004286
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
                                "x": -2.168138977591132,
                                "y": 2.121801814815506
                            },
                            {
                                "x": -2.001954330748212,
                                "y": 1.6023950501567528
                            },
                            {
                                "x": -2.218326362814403,
                                "y": 1.533166630000868
                            },
                            {
                                "x": -2.384511009657323,
                                "y": 2.0525733946596207
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": -2.1932326702027676,
                            "y": 1.8274842224081869
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
                                "x": 0.10408812129141666,
                                "y": 1.612005166200347
                            },
                            {
                                "x": 0.3816479730062513,
                                "y": 1.1872233576761473
                            },
                            {
                                "x": 0.19799963101013873,
                                "y": 1.0672243239387778
                            },
                            {
                                "x": -0.0795602207046959,
                                "y": 1.4920061324629774
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": 0.15104387615077772,
                            "y": 1.3396147450695624
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
                                "x": 3.6389517717869366,
                                "y": -1.8639552365117376
                            },
                            {
                                "x": 4.260563375001744,
                                "y": -1.8063761043369368
                            },
                            {
                                "x": 4.310548577467738,
                                "y": -2.346005318399705
                            },
                            {
                                "x": 3.6889369742529317,
                                "y": -2.403584450574506
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": 3.9747501746273377,
                            "y": -2.1049802774557214
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
                                "x": -1.5645766815263709,
                                "y": 4.476229324644058
                            },
                            {
                                "x": -1.148730357827309,
                                "y": 4.115802045399985
                            },
                            {
                                "x": -1.2938536956277131,
                                "y": 3.948364645451219
                            },
                            {
                                "x": -1.7097000193267753,
                                "y": 4.308791924695293
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": -1.4292151885770419,
                            "y": 4.212296985047638
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
                                "x": -0.8514831739879747,
                                "y": -2.543899419075606
                            },
                            {
                                "x": -0.4443265502801087,
                                "y": -1.9617458275925337
                            },
                            {
                                "x": -0.19176303458125177,
                                "y": -2.138388057114406
                            },
                            {
                                "x": -0.5989196582891176,
                                "y": -2.7205416485974783
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": -0.5216231042846131,
                            "y": -2.3411437380950058
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
                                "x": -2.804032408332575,
                                "y": 2.112090278542337
                            },
                            {
                                "x": -2.8525076324207252,
                                "y": 1.5590739915902232
                            },
                            {
                                "x": -3.3636228565961153,
                                "y": 1.6038763339648388
                            },
                            {
                                "x": -3.315147632507965,
                                "y": 2.1568926209169526
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": -3.083827632464345,
                            "y": 1.8579833062535878
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
                                "x": 2.5574334188949663,
                                "y": -1.7752616685163014
                            },
                            {
                                "x": 3.2986423144771346,
                                "y": -2.010564746616259
                            },
                            {
                                "x": 3.0955737562644563,
                                "y": -2.650234329248504
                            },
                            {
                                "x": 2.354364860682288,
                                "y": -2.4149312511485466
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": 2.8265035875797113,
                            "y": -2.212747998882403
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
                                "x": -0.46336623697415197,
                                "y": 0.20083398971672267
                            },
                            {
                                "x": -1.045576177942429,
                                "y": -0.12331584207274762
                            },
                            {
                                "x": -1.1835249385171103,
                                "y": 0.12445581126253914
                            },
                            {
                                "x": -0.6013149975488332,
                                "y": 0.44860564305200945
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": -0.8234455877456311,
                            "y": 0.1626449004896309
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
                                "x": 2.318098339135203,
                                "y": 0.3088983984089546
                            },
                            {
                                "x": 1.6511700174939397,
                                "y": 0.3097237212932523
                            },
                            {
                                "x": 1.6518761600385474,
                                "y": 0.8803445911065477
                            },
                            {
                                "x": 2.318804481679811,
                                "y": 0.87951926822225
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": 1.9849872495868752,
                            "y": 0.5946214947577513
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
                                "x": 2.9491598889652595,
                                "y": 1.0486941946808972
                            },
                            {
                                "x": 2.4138558436859276,
                                "y": 1.6041487453646475
                            },
                            {
                                "x": 2.7240356560007397,
                                "y": 1.9030760078953282
                            },
                            {
                                "x": 3.2593397012800716,
                                "y": 1.3476214572115779
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": 2.8365977724829996,
                            "y": 1.4758851012881127
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
                                "x": -2.673021892450052,
                                "y": 0.7625134973059582
                            },
                            {
                                "x": -3.1513696376956073,
                                "y": 1.4520017416865747
                            },
                            {
                                "x": -2.8706317460778283,
                                "y": 1.6467698737828538
                            },
                            {
                                "x": -2.392284000832273,
                                "y": 0.9572816294022374
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": -2.77182681926394,
                            "y": 1.204641685544406
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
                                "x": 0.9369257797047466,
                                "y": 0.09048603888839418
                            },
                            {
                                "x": 1.7638821669592666,
                                "y": -0.06745505448134517
                            },
                            {
                                "x": 1.6524909418182365,
                                "y": -0.6506831628168928
                            },
                            {
                                "x": 0.8255345545637165,
                                "y": -0.4927420694471533
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": 1.2947083607614915,
                            "y": -0.28009856196424926
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
                                "x": 2.769064486406439,
                                "y": 3.9077918512462895
                            },
                            {
                                "x": 3.37687723816771,
                                "y": 3.9993718524475192
                            },
                            {
                                "x": 3.4450635017091242,
                                "y": 3.546822388243432
                            },
                            {
                                "x": 2.837250749947853,
                                "y": 3.4552423870422024
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": 3.1070639940577816,
                            "y": 3.727307119744861
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
                                "x": -0.4891909929285043,
                                "y": 4.282469945376864
                            },
                            {
                                "x": -0.2134985027293859,
                                "y": 5.107494052341588
                            },
                            {
                                "x": 0.41746528200498056,
                                "y": 4.896649332083838
                            },
                            {
                                "x": 0.14177279180586208,
                                "y": 4.071625225119114
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": -0.035862855461761906,
                            "y": 4.589559638730351
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
                                "x": -3.6976248307416033,
                                "y": -1.9208164856817014
                            },
                            {
                                "x": -3.4725410234899723,
                                "y": -1.552761203888247
                            },
                            {
                                "x": -3.094141466430782,
                                "y": -1.7841710414272995
                            },
                            {
                                "x": -3.319225273682413,
                                "y": -2.152226323220754
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": -3.3958831485861927,
                            "y": -1.8524937635545005
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
                                "x": 1.4165216734094386,
                                "y": 1.4510314101525603
                            },
                            {
                                "x": 2.151676502061771,
                                "y": 1.5359127916710134
                            },
                            {
                                "x": 2.1870598720925027,
                                "y": 1.2294585979862669
                            },
                            {
                                "x": 1.4519050434401706,
                                "y": 1.1445772164678139
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": 1.8017907727509708,
                            "y": 1.3402450040694136
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
                                "x": -4.3562564845440095,
                                "y": 1.805073514351832
                            },
                            {
                                "x": -4.50665071221233,
                                "y": 2.280371059771919
                            },
                            {
                                "x": -4.1000689901939085,
                                "y": 2.4090221429854277
                            },
                            {
                                "x": -3.9496747625255884,
                                "y": 1.9337245975653405
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": -4.228162737368959,
                            "y": 2.10704782866863
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
                                "x": -2.646102420073309,
                                "y": 4.624903310910418
                            },
                            {
                                "x": -2.120881884818803,
                                "y": 4.010138524726749
                            },
                            {
                                "x": -2.6340259889454485,
                                "y": 3.5717369976887805
                            },
                            {
                                "x": -3.1592465241999546,
                                "y": 4.186501783872449
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": -2.6400642045093785,
                            "y": 4.098320154299599
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
                                "x": -0.226885189454957,
                                "y": 1.494907356637786
                            },
                            {
                                "x": -0.40197132312813755,
                                "y": 0.9676209074996082
                            },
                            {
                                "x": -0.7969478361548425,
                                "y": 1.098773359208903
                            },
                            {
                                "x": -0.6218617024816621,
                                "y": 1.626059808347081
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": -0.5119165128048998,
                            "y": 1.2968403579233445
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
                                "x": 3.7074212452981126,
                                "y": 2.1189283772894907
                            },
                            {
                                "x": 3.406446759489361,
                                "y": 2.6575339697605536
                            },
                            {
                                "x": 3.9654326793438237,
                                "y": 2.96989704610861
                            },
                            {
                                "x": 4.266407165152575,
                                "y": 2.431291453637547
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": 3.836426962320968,
                            "y": 2.5444127116990503
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
                                "x": -3.3943890179835496,
                                "y": 1.0622460717547981
                            },
                            {
                                "x": -2.6971660838002394,
                                "y": 1.0516049657930808
                            },
                            {
                                "x": -2.702529749893189,
                                "y": 0.7001686559997335
                            },
                            {
                                "x": -3.3997526840764993,
                                "y": 0.7108097619614508
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": -3.0484593839383693,
                            "y": 0.8812073638772658
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
                                "x": 3.147952230642958,
                                "y": -0.8233581037016315
                            },
                            {
                                "x": 3.113332438276919,
                                "y": 0.014215875755012153
                            },
                            {
                                "x": 3.8015928228948965,
                                "y": 0.042664027433150675
                            },
                            {
                                "x": 3.8362126152609357,
                                "y": -0.794909952023493
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": 3.4747725267689273,
                            "y": -0.3903470381342405
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
                                "x": 0.0721724530675486,
                                "y": 1.13036412441174
                            },
                            {
                                "x": 0.8002279113814715,
                                "y": 1.4389233491696558
                            },
                            {
                                "x": 1.0286899512757008,
                                "y": 0.8998597859492545
                            },
                            {
                                "x": 0.300634492961778,
                                "y": 0.5913005611913386
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": 0.5504312021716248,
                            "y": 1.0151119551804972
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
                                "x": -3.8900892802531115,
                                "y": 0.27057220056045966
                            },
                            {
                                "x": -4.655262013669786,
                                "y": 0.45346455295047505
                            },
                            {
                                "x": -4.512051425675963,
                                "y": 1.0526193871327325
                            },
                            {
                                "x": -3.7468786922592887,
                                "y": 0.8697270347427171
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": -4.201070352964537,
                            "y": 0.6615957938465961
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
                                "x": 2.297364968534206,
                                "y": -2.299622518880312
                            },
                            {
                                "x": 1.9001948626031064,
                                "y": -1.7178596386759384
                            },
                            {
                                "x": 2.277529251237015,
                                "y": -1.4602530704250931
                            },
                            {
                                "x": 2.674699357168115,
                                "y": -2.0420159506294664
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": 2.2874471098856106,
                            "y": -1.8799377946527025
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
                                "x": -0.4278616982520657,
                                "y": -0.49381114451806796
                            },
                            {
                                "x": 0.23146061630020237,
                                "y": -1.0728397366154594
                            },
                            {
                                "x": 0.03390759584003028,
                                "y": -1.2977873755995535
                            },
                            {
                                "x": -0.6254147187122379,
                                "y": -0.7187587835021619
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": -0.19697705120601772,
                            "y": -0.8957992600588107
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
                                "x": -0.9936440078081616,
                                "y": 0.29104615554776636
                            },
                            {
                                "x": -1.3938443515073469,
                                "y": -0.49806297711137615
                            },
                            {
                                "x": -1.654218569149833,
                                "y": -0.36601298924747994
                            },
                            {
                                "x": -1.2540182254506478,
                                "y": 0.4230961434116626
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": -1.3239312884789973,
                            "y": -0.03748341684985679
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
                                "x": 4.1879712406231775,
                                "y": 1.5487752548799048
                            },
                            {
                                "x": 3.993329710952244,
                                "y": 2.0594120457311074
                            },
                            {
                                "x": 4.251102443999471,
                                "y": 2.1576683405510195
                            },
                            {
                                "x": 4.445743973670405,
                                "y": 1.647031549699817
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": 4.219536842311324,
                            "y": 1.8532217977154621
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
                                "x": 3.23730457554032,
                                "y": -0.6924081083151619
                            },
                            {
                                "x": 3.3335495207105286,
                                "y": -0.1151082874010082
                            },
                            {
                                "x": 3.8036378719505013,
                                "y": -0.19347939709337192
                            },
                            {
                                "x": 3.707392926780292,
                                "y": -0.7707792180075257
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": 3.5204712237454103,
                            "y": -0.44294375270426695
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
                                "x": 0.5789858973826981,
                                "y": 3.742275219649017
                            },
                            {
                                "x": -0.19876535731169667,
                                "y": 3.5274118778971424
                            },
                            {
                                "x": -0.3760198505681239,
                                "y": 4.169028560419036
                            },
                            {
                                "x": 0.40173140412627084,
                                "y": 4.38389190217091
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": 0.10148302340728707,
                            "y": 3.955651890034026
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
                                "x": -2.3926769878288163,
                                "y": -0.8821399730238485
                            },
                            {
                                "x": -2.2472882586000162,
                                "y": -0.42765309093896176
                            },
                            {
                                "x": -2.0011384266957237,
                                "y": -0.5063955424180423
                            },
                            {
                                "x": -2.146527155924524,
                                "y": -0.960882424502929
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": -2.19690770726227,
                            "y": -0.6942677577209454
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
                                "x": 0.5289699585001228,
                                "y": -1.8553292648563575
                            },
                            {
                                "x": 1.2234373031352217,
                                "y": -1.4787005892971732
                            },
                            {
                                "x": 1.3977591575573425,
                                "y": -1.8001334572875254
                            },
                            {
                                "x": 0.7032918129222436,
                                "y": -2.1767621328467097
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": 0.9633645580287327,
                            "y": -1.8277313610719415
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
                                "x": -3.160654596459941,
                                "y": 3.0510360869770294
                            },
                            {
                                "x": -3.655545490550616,
                                "y": 3.412085287663927
                            },
                            {
                                "x": -3.259105377880906,
                                "y": 3.9554865787878515
                            },
                            {
                                "x": -2.764214483790231,
                                "y": 3.5944373781009538
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": -3.2098799871704236,
                            "y": 3.5032613328824405
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
                                "x": -1.5183103687980364,
                                "y": -0.7117401454542095
                            },
                            {
                                "x": -1.0707711116763876,
                                "y": -0.34847644087015683
                            },
                            {
                                "x": -0.7733422165115458,
                                "y": -0.7149075076374853
                            },
                            {
                                "x": -1.2208814736331943,
                                "y": -1.078171212221538
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": -1.145826292654791,
                            "y": -0.7133238265458474
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
                                "x": 2.763062309116752,
                                "y": 3.173082212662751
                            },
                            {
                                "x": 3.04738144840952,
                                "y": 3.501430147248112
                            },
                            {
                                "x": 3.2363586305412286,
                                "y": 3.337793272731114
                            },
                            {
                                "x": 2.9520394912484607,
                                "y": 3.009445338145753
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": 2.9997104698289903,
                            "y": 3.2554377426969325
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
                                "x": -1.6472140771778916,
                                "y": -0.9819834283077002
                            },
                            {
                                "x": -1.8547968867863311,
                                "y": -0.5393723435540051
                            },
                            {
                                "x": -1.4788613134257762,
                                "y": -0.36306007927040673
                            },
                            {
                                "x": -1.2712785038173366,
                                "y": -0.8056711640241019
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": -1.563037695301834,
                            "y": -0.6725217537890534
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
                                "x": -1.4400862383645918,
                                "y": 4.482148983190662
                            },
                            {
                                "x": -1.9289715656265893,
                                "y": 4.568945054723974
                            },
                            {
                                "x": -1.8820891167592781,
                                "y": 4.833013934253188
                            },
                            {
                                "x": -1.3932037894972806,
                                "y": 4.746217862719876
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": -1.661087677561935,
                            "y": 4.657581458721925
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
                                "x": 4.376813478041394,
                                "y": 0.5501876247569756
                            },
                            {
                                "x": 4.657069416825062,
                                "y": 1.3723259690624308
                            },
                            {
                                "x": 5.1628061314028555,
                                "y": 1.1999271037095367
                            },
                            {
                                "x": 4.882550192619187,
                                "y": 0.3777887594040815
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": 4.769809804722125,
                            "y": 0.8750573642332562
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
                                "x": -3.9610224294263268,
                                "y": -0.5367898449399351
                            },
                            {
                                "x": -3.140131825496806,
                                "y": -0.3343985690867329
                            },
                            {
                                "x": -3.0603578775054596,
                                "y": -0.6579583869118668
                            },
                            {
                                "x": -3.8812484814349806,
                                "y": -0.8603496627650691
                            }
                        ],
                        "pose": {
                            "theta": 0,
                            "x": -3.510690153465893,
                            "y": -0.597374115925901
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

    # Draw the quad tree
    # quad_tree.draw()

    # Remove a polygon from the quad tree
    # quad_tree.remove(2)

    # Query the polygon
    from model.geometry.segment import Segment
    start = Point(-0.22055705893163785, 0.6120829290731538)
    end = Point(-0.14347041355998716, 0.7966300750964541)
    segment = Segment(start, end)
    query_polygon = Polygon.segment_buffer(segment, 0.2, 0.2)
    query_bounds = query_polygon.get_bounds()

    result = quad_tree.query_region(query_bounds)
    print(f'Result: {result}')
