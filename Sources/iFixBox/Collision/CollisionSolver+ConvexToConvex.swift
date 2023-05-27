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
        
        // Normal is always look at A * <-| * B
        
        let isInA = a.points.count > b.points.count
        let iMt: Transform
        let dMt: Transform
        let polyA: [FixVec]
        let polyB: [FixVec]
        let bndA: Boundary
        let bndB: Boundary
        
        if a.points.count > b.points.count {
            polyA = a.points
            bndA = a.boundary
            
            dMt = tA
            iMt = Transform.convertFromBtoA(tB, tA)
            polyB = iMt.convertAsPoints(b.points)
            bndB = Boundary(points: polyB)
        } else {
            polyB = b.points
            bndB = b.boundary
            
            dMt = tB
            iMt = Transform.convertFromBtoA(tA, tB)
            polyA = iMt.convertAsPoints(a.points)
            bndA = Boundary(points: polyA)
        }
        
        let pins = OverlaySolver.find(polyA: polyA, polyB: polyB, bndA: bndA, bndB: bndB)

        var n: FixVec = .zero
        
        switch pins.count {
        case 0, 1:
            if bndA.isOverlap(bndB) {
                n = (tA.position - tB.position).safeNormalize()
                
                let contact = Contact(
                    point: tB.position,
                    normal: n,
                    penetration: -b.radius,
                    status: .inside,
                    type: .average
                )
                
                return contact
            } else if bndB.isOverlap(bndA) {
                n = (tA.position - tB.position).safeNormalize()
                
                let contact = Contact(
                    point: tA.position,
                    normal: n,
                    penetration: -a.radius,
                    status: .inside,
                    type: .average
                )
                
                return contact
            } else if pins.count == 1 {
                let pin = pins[0]
                if pin.mA.offset == 0 && pin.mB.offset == 0 {
                    // vertex - vertex
                    n = (tA.position - tB.position).safeNormalize()
                } else if pin.mA.offset == 0 {
                    // vertex is A
                    let e = b.normals[pin.mB.index]
                    n = isInA ? iMt.convertAsVector(e) : e
                } else {
                    // vertex is B
                    let e = a.normals[pin.mA.index].negative
                    n = isInA ? e : iMt.convertAsVector(e)
                }
                let contact = Contact(
                    point: pin.p,
                    normal: n,
                    penetration: 0,
                    status: .collide,
                    type: .vertex
                )
                
                return dMt.convert(contact)
            } else {
                return .outside
            }
        default:
            let centroid = OverlaySolver.intersect(polyA: polyA, polyB: polyB, pins: pins, bndA: bndA, bndB: bndB)
            
            if centroid.area > a.radius.sqr || centroid.area > b.radius.sqr {
                n = (tA.position - tB.position).safeNormalize()
                if a.radius > b.radius  {
                    // A overlap B
                    
                    let contact = Contact(
                        point: tB.position,
                        normal: n,
                        penetration: -b.radius,
                        status: .inside,
                        type: .average
                    )
                    
                    return contact
                } else {
                    // B overlap A
                    
                    let contact = Contact(
                        point: tA.position,
                        normal: n,
                        penetration: -a.radius,
                        status: .inside,
                        type: .average
                    )
                    
                    return contact
                }
            }
            
            if pins.count == 2 {
                let p0 = pins[0]
                let p1 = pins[1]
                let ai = MileStone.sameEdgeIndex(polyA.count, m0: p0.mA, m1: p1.mA)
                let bi = MileStone.sameEdgeIndex(polyB.count, m0: p0.mB, m1: p1.mB)

                let pen: FixFloat
                if centroid.area > 0 {
                    let dist = p0.p.distance(p1.p)
                    if dist != 0 {
                        pen = -centroid.area.div(2 * dist)
                    } else {
                        pen = -centroid.area.sqrt
                    }
                } else {
                    pen = 0
                }
                
                if ai >= 0 && bi >= 0 {
                    
                    // take "local coord system" normals
                    if isInA {
                        n = a.normals[ai].negative
                    } else {
                        n = b.normals[bi]
                    }
                    
                    let contact = Contact(
                        point: centroid.center,
                        normal: n,
                        penetration: pen,
                        status: .collide,
                        type: .edge
                    )
                    
                    return dMt.convert(contact)
                } else if ai >= 0 {
                    if isInA {
                        n = a.normals[ai].negative
                    } else {
                        n = iMt.convertAsVector(a.normals[ai].negative)
                    }
                    
                    let contact = Contact(
                        point: centroid.center,
                        normal: n,
                        penetration: pen,
                        status: .collide,
                        type: .edge
                    )
                    
                    return dMt.convert(contact)
                } else if bi >= 0 {
                    if isInA {
                        n = iMt.convertAsVector(b.normals[bi])
                    } else {
                        n = b.normals[bi]
                    }
                    let contact = Contact(
                        point: centroid.center,
                        normal: n,
                        penetration: pen,
                        status: .collide,
                        type: .edge
                    )
                    
                    return dMt.convert(contact)
                } else {
                    n = (tA.position - tB.position).safeNormalize()

                    let contact = Contact(
                        point: dMt.convertAsPoint(centroid.center),
                        normal: n,
                        penetration: pen,
                        status: .collide,
                        type: .average
                    )
                    
                    return contact
                }
            } else {
                let pen = centroid.area > 0 ? -(centroid.area.sqrt >> 1) : 0
                n = (tA.position - tB.position).safeNormalize()
                let contact = Contact(
                    point: dMt.convertAsPoint(centroid.center),
                    normal: n,
                    penetration: pen,
                    status: .collide,
                    type: .average
                )
                
                return contact
            }
        }
    }
}
