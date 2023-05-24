//
//  CollisionSolver+ConvexToConvex.swift
//  
//
//  Created by Nail Sharipov on 09.05.2023.
//

import iFixFloat
import iConvex

extension CollisionSolver {

    public func collide(_ a: ConvexCollider, _ b: ConvexCollider, tA: Transform, tB: Transform) -> Contact {
        let isInA = a.points.count > b.points.count
        let mt: Transform
        let polyA: [FixVec]
        let polyB: [FixVec]
        let bndA: Boundary
        let bndB: Boundary
        
        if a.points.count > b.points.count {
            polyA = a.points
            bndA = a.boundary
            
            mt = Transform.convertFromBtoA(tB, tA)
            polyB = mt.convertAsPoints(b.points)
            bndB = Boundary(points: polyB)
        } else {
            polyB = b.points
            bndB = b.boundary
            
            mt = Transform.convertFromBtoA(tA, tB)
            polyA = mt.convertAsPoints(a.points)
            bndA = Boundary(points: polyA)
        }
        
        let pins = OverlaySolver.find(polyA: polyA, polyB: polyB, bndA: bndA, bndB: bndB)

        var n: FixVec = .zero
        
        switch pins.count {
        case 0, 1:
            if bndA.isOverlap(bndB) {
                return self.collide(b.circleCollider, a).negativeNormal()
            } else if bndB.isOverlap(bndA) {
                return self.collide(a.circleCollider, b)
            } else if pins.count == 1 {
                let pin = pins[0]
                if pin.mA.offset == 0 && pin.mB.offset == 0 {
                    // vertex - vertex
                    n = (b.center - a.center).safeNormalize()
                } else if pin.mA.offset == 0 {
                    // vertex is A
                    let e = b.normals[pin.mB.index].negative
                    n = isInA ? mt.convertAsVector(e) : e
                } else {
                    // vertex is B
                    let e = a.normals[pin.mA.index]
                    n = isInA ? e : mt.convertAsVector(e)
                }
                let contact = Contact(
                    point: pin.p,
                    normal: n,
                    penetration: 0,
                    status: .collide,
                    type: .vertex
                )
                
                return isInA ? tA.convert(contact) : tB.convert(contact)
            } else {
                return .outside
            }
        default:
            let centroid = OverlaySolver.intersect(polyA: polyA, polyB: polyB, pins: pins, bndA: bndA, bndB: bndB)
            
            let contact: Contact
            
            if pins.count == 2 {
                let p0 = pins[0]
                let p1 = pins[1]
                let ai = MileStone.sameEdgeIndex(polyA.count, m0: p0.mA, m1: p1.mA)
                let bi = MileStone.sameEdgeIndex(polyB.count, m0: p0.mB, m1: p1.mB)

                if ai >= 0 && bi >= 0 {
                    if isInA {
                        n = a.normals[ai]
                    } else {
                        n = b.normals[bi].negative
                    }

                    contact = Contact(
                        point: centroid.center,
                        normal: n,
                        penetration: 0,
                        status: .collide,
                        type: .edge
                    )
                } else if ai >= 0 {
                    if isInA {
                        n = a.normals[ai]
                    } else {
                        n = mt.convertAsVector(a.normals[ai])
                    }
                    
                    contact = Contact(
                        point: centroid.center,
                        normal: n,
                        penetration: 0,
                        status: .collide,
                        type: .edge
                    )
                } else if bi >= 0 {
                    if isInA {
                        n = mt.convertAsVector(b.normals[bi].negative)
                    } else {
                        n = b.normals[bi].negative
                    }
                    contact = Contact(
                        point: centroid.center,
                        normal: n,
                        penetration: 0,
                        status: .collide,
                        type: .edge
                    )
                } else {
                    let t = (p0.p - p1.p).safeNormalize()
                    n = FixVec(t.y, -t.x)
                    if (b.center - a.center).unsafeDotProduct(n) < 0 {
                        n = n.negative
                    }

                    contact = Contact(
                        point: centroid.center,
                        normal: n,
                        penetration: centroid.area > 0 ? -(centroid.area.sqrt >> 1) : 0,
                        status: .collide,
                        type: .edge
                    )
                }
            } else {
                n = (b.center - a.center).safeNormalize()
                contact = Contact(
                    point: centroid.center,
                    normal: n,
                    penetration: centroid.area > 0 ? -(centroid.area.sqrt >> 1) : 0,
                    status: .collide,
                    type: .vertex
                )
            }
            
            return isInA ? tA.convert(contact) : tB.convert(contact)
        }
        
    }
}
