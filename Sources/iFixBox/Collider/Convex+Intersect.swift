//
//  Convex+Intersect.swift
//  
//
//  Created by Nail Sharipov on 15.05.2023.
//

import iFixFloat

private struct Edge {
    let p0: FixVec
    let p1: FixVec
    let i0: Int
    let i1: Int
}

public struct Dot {
    public let p: FixVec
}


public struct ConvexIntersectSolver {
    
    
    public static func intersect(a: ConvexCollider, b: ConvexCollider) -> [Dot] {
        let edAs = a.sortedEdges(filter: b.boundary)
        let edBs = b.sortedEdges(filter: a.boundary)
        
        guard !edAs.isEmpty && !edBs.isEmpty else {
            return []
        }

        var i0 = 0
        var dots = [Int64 : Dot]()
        dots.reserveCapacity(4)

        for edB in edBs {
            
            var i = i0
            while i < edAs.count {
                let edA = edAs[i]
                
                if edB.p0.bitPack > edA.p1.bitPack {
                    i0 += 1
                } else {
                    let cross = edA.cross(other: edB)
                    switch cross.type {
                    case .not_cross, .same_line:
                        break
                    case .pure:
                        dots[cross.point.bitPack] = Dot(p: cross.point)
                    case .end_a0:
                        dots[cross.point.bitPack] = Dot(p: cross.point)
                    case .end_a1:
                        dots[cross.point.bitPack] = Dot(p: cross.point)
                    case .end_b0:
                        dots[cross.point.bitPack] = Dot(p: cross.point)
                    case .end_b1:
                        dots[cross.point.bitPack] = Dot(p: cross.point)
                    case .end_a0_b0:
                        dots[cross.point.bitPack] = Dot(p: cross.point)
                    case .end_a0_b1:
                        dots[cross.point.bitPack] = Dot(p: cross.point)
                    case .end_a1_b0:
                        dots[cross.point.bitPack] = Dot(p: cross.point)
                    case .end_a1_b1:
                        dots[cross.point.bitPack] = Dot(p: cross.point)
                    }
                }
                i += 1
            }
        }

//        print(dots)
        
        return Array(dots.values)
    }
    
}

private extension ConvexCollider {
    
    var mostLeft: Int {
        let n = points.count
        let p1 = points[1].bitPack
        let p0 = points[0].bitPack
        if p1 > p0 {
            // search back
            var p = p0
            var i = 0
            var j = i.prev(n)
            var pi = points[j].bitPack
            while pi < p {
                p = pi
                i = j
                j = j.prev(n)
                pi = points[j].bitPack
            }
            return i
        } else {
            // search forward
            var p = p1
            var i = 1
            var j = i.next(n)
            var pi = points[j].bitPack
            while pi < p {
                p = pi
                i = j
                j = j.next(n)
                pi = points[j].bitPack
            }
            return i
        }
    }
    
    func sortedEdges(filter boundary: Boundary) -> [Edge] {
        let left = self.mostLeft
        let n = points.count
        
        // i will be direct index, j will be reverse index

        var edges = [Edge]()
        edges.reserveCapacity(n / 2)
        
        var i0 = left
        var j0 = left
        
        var pi0 = points[i0]
        var pj0 = pi0

        repeat {
            let edge: Edge
            if pi0.bitPack < pj0.bitPack {
                let i1 = i0.next(n)
                let pi1 = points[i1]
                edge = Edge(p0: pi0, p1: pi1, i0: i0, i1: i1)

                i0 = i1
                pi0 = pi1
            } else {
                let j1 = j0.prev(n)
                let pj1 = points[j1]
                edge = Edge(p0: pj0, p1: pj1, i0: j0, i1: j1)

                j0 = j1
                pj0 = pj1
            }
            
            if boundary.isIntersectionPossible(edge: edge) {
                edges.append(edge)
            }
            
        } while i0 != j0
        
        return edges
    }
}


private extension Int {
    
    func next(_ n: Int) -> Int {
        let x = self + 1
        return x == n ? 0 : x
    }
    
    func prev(_ n: Int) -> Int {
        let x = self - 1
        return x < 0 ? (n - 1) : x
    }
}

private extension Boundary {
 
    func isIntersectionPossible(edge e: Edge) -> Bool {
        // p1.x > p0.x by it sorted nature
        let eMinX = e.p0.x
        let eMaxX = e.p1.x
        
        let eMinY: Int64
        let eMaxY: Int64
        if e.p0.y > e.p1.y {
            eMinY = e.p1.y
            eMaxY = e.p0.y
        } else {
            eMinY = e.p0.y
            eMaxY = e.p1.y
        }
        
        let c0 = eMaxX < min.x
        let c1 = eMinX > max.x
        let c2 = eMaxY < min.y
        let c3 = eMinY > max.y
        
        return !(c0 || c1 || c2 || c3)
    }
    
}


private extension Edge {

    struct CrossResult {
        let type: CrossType
        let point: FixVec
    }
    
    enum CrossType {
        case not_cross          // no intersections
        case pure               // simple intersection with no overlaps or common points
        case same_line          // same line
        
        case end_a0
        case end_a1
        case end_b0
        case end_b1
        case end_a0_b0
        case end_a0_b1
        case end_a1_b0
        case end_a1_b1
        
    }
    
    func cross(other: Edge) -> CrossResult {
        let a0 = self.p0
        let a1 = self.p1

        let b0 = other.p0
        let b1 = other.p1
        
        let d0 = self.isCCW(a: a0, b: b0, c: b1)
        let d1 = self.isCCW(a: a1, b: b0, c: b1)
        let d2 = self.isCCW(a: a0, b: a1, c: b0)
        let d3 = self.isCCW(a: a0, b: a1, c: b1)

        if d0 == 0 || d1 == 0 || d2 == 0 || d3 == 0 {
            if d0 == 0 && d1 == 0 && d2 == 0 && d3 == 0 {
                return .init(type: .same_line, point: .zero)
            }
            if d0 == 0 {
                if d2 == 0 || d3 == 0 {
                    if d2 == 0 {
                        return .init(type: .end_a0_b0, point: a0)
                    } else {
                        return .init(type: .end_a0_b1, point: a0)
                    }
                } else if d2 != d3 {
                    return .init(type: .end_a0, point: a0)
                } else {
                    return .init(type: .not_cross, point: .zero)
                }
            }
            if d1 == 0 {
                if d2 == 0 || d3 == 0 {
                    if d2 == 0 {
                        return .init(type: .end_a1_b0, point: a1)
                    } else {
                        return .init(type: .end_a1_b1, point: a1)
                    }
                } else if d2 != d3 {
                    return .init(type: .end_a1, point: a1)
                } else {
                    return .init(type: .not_cross, point: .zero)
                }
            }
            if d0 != d1 {
                if d2 == 0 {
                    return .init(type: .end_b0, point: b0)
                } else {
                    return .init(type: .end_b1, point: b1)
                }
            } else {
                return .init(type: .not_cross, point: .zero)
            }
        } else if d0 != d1 && d2 != d3 {
            let cross = self.crossPoint(a0: a0, a1: a1, b0: b0, b1: b1)

            // still can be ends
            let isA0 = a0 == cross
            let isA1 = a1 == cross
            let isB0 = b0 == cross
            let isB1 = b1 == cross
            
            let type: CrossType
            
            if !(isA0 || isA1 || isB0 || isB1) {
                type = .pure
            } else if isA0 && isB0 {
                type = .end_a0_b0
            } else if isA0 && isB1 {
                type = .end_a0_b1
            } else if isA1 && isB0 {
                type = .end_a1_b0
            } else if isA1 && isB1 {
                type = .end_a1_b1
            } else if isA0 {
                type = .end_a0
            } else if isA1 {
                type = .end_a1
            } else if isB0 {
                type = .end_b0
            } else {
                type = .end_b1
            }
            
            return .init(type: type, point: cross)
        } else {
            return .init(type: .not_cross, point: .zero)
        }
    }
    
    private func isCCW(a: FixVec, b: FixVec, c: FixVec) -> Int {
        let m0 = (c.y - a.y) * (b.x - a.x)
        let m1 = (b.y - a.y) * (c.x - a.x)

        if m0 < m1 {
            return -1
        }
        
        if m0 > m1 {
            return 1
        }

        return 0
    }
    
    private func crossPoint(a0: FixVec, a1: FixVec, b0: FixVec, b1: FixVec) -> FixVec {
        let dxA = a0.x - a1.x
        let dyB = b0.y - b1.y
        let dyA = a0.y - a1.y
        let dxB = b0.x - b1.x
        
        let divider = dxA.mul(dyB) - dyA.mul(dxB)
        let invert_divider = divider.doubleInvert
        
        let xyA = a0.x.mul(a1.y) - a0.y.mul(a1.x)
        let xyB = b0.x.mul(b1.y) - b0.y.mul(b1.x)
        
        let x = xyA.mul(b0.x - b1.x) - (a0.x - a1.x).mul(xyB)
        let y = xyA.mul(b0.y - b1.y) - (a0.y - a1.y).mul(xyB)

        let cx = x.mul(fixDouble: invert_divider)
        let cy = y.mul(fixDouble: invert_divider)

        return FixVec(cx, cy)
    }
}
