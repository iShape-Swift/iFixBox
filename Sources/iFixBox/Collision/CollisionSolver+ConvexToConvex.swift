//
//  CollisionSolver+ConvexToConvex.swift
//  
//
//  Created by Nail Sharipov on 09.05.2023.
//

import iShape
import iConvex
import iFixFloat

extension CollisionSolver {

    public func collide(_ a: ConvexCollider, _ b: ConvexCollider, tA: Transform, tB: Transform) -> Contact {
        
        // Normal is always look at A * <-| * B
        
        let isInA = a.points.count > b.points.count
        let iMt: Transform
        let dMt: Transform
        let polyA: [FixVec]
        let polyB: [FixVec]
        let bndA: FixBnd
        let bndB: FixBnd
        
        if a.points.count > b.points.count {
            polyA = a.points
            bndA = a.boundary
            
            dMt = tA
            iMt = Transform.convertFromBtoA(tB, tA)
            polyB = iMt.convertAsPoints(b.points)
            bndB = FixBnd(points: polyB)
        } else {
            polyB = b.points
            bndB = b.boundary
            
            dMt = tB
            iMt = Transform.convertFromBtoA(tA, tB)
            polyA = iMt.convertAsPoints(a.points)
            bndA = FixBnd(points: polyA)
        }
        
        let pins = OverlaySolver.find(pathA: polyA, pathB: polyB, bndA: bndA, bndB: bndB)

        var n: FixVec = .zero
        
        switch pins.count {
        case 0, 1:
            if bndA.isOverlap(bndB) {
                n = (tA.position - tB.position).fixNormalize
                
                let contact = Contact(
                    point: tB.position,
                    normal: n,
                    penetration: -b.radius,
                    status: .inside,
                    type: .average
                )
                
                return contact
            } else if bndB.isOverlap(bndA) {
                n = (tA.position - tB.position).fixNormalize
                
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
                    n = (tA.position - tB.position).fixNormalize
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
            let centroid = OverlaySolver.intersect(pathA: polyA, pathB: polyB, pins: pins, bndA: bndA, bndB: bndB)
            
            if centroid.area > 3 * a.radius.fixSqr || centroid.area > 3 * b.radius.fixSqr {
                n = (tA.position - tB.position).fixNormalize
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
                let p0 = dMt.convertAsPoint(pins[0].p)
                let p1 = dMt.convertAsPoint(pins[1].p)
                
                if p0 != p1 {
                    let slice = p0 - p1
                    let point = dMt.convertAsPoint(centroid.center)
                    let pen: FixFloat = centroid.area > 0 ? -centroid.area.fixDiv(slice.fixLength) : 0

                    n = FixVec(slice.y, -slice.x).fixNormalize
                    
                    if n.fixDotProduct(tA.position - point) < 0 {
                        n = n.negative
                    }
                    
                    let contact = Contact(
                        point: point,
                        normal: n,
                        penetration: pen,
                        status: .collide,
                        type: .edge
                    )
                    
                    return contact
                } else {
                    return Contact(
                        point: dMt.convertAsPoint(centroid.center),
                        normal: (tA.position - tB.position).fixNormalize,
                        penetration: 0,
                        status: .collide,
                        type: .average
                    )
                }
            } else {
                let pen = centroid.area > 0 ? -(centroid.area.fixSqrt >> 1) : 0
                n = (tA.position - tB.position).fixNormalize
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
