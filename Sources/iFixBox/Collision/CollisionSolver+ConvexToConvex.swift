//
//  CollisionSolver+ConvexToConvex.swift
//  
//
//  Created by Nail Sharipov on 09.05.2023.
//

import iFixFloat

extension CollisionSolver {

    func collide(_ a: ConvexCollider, _ b: ConvexCollider, tA: Transform, tB: Transform) -> Contact {
        if a.points.count > b.points.count {
            let mt = Transform.convertFromBtoA(tB, tA)
            
            let b2 = ConvexCollider(transform: mt, collider: b)

            let contact = collide(a, b2)

            return tA.convert(contact)
        } else {
            let mt = Transform.convertFromBtoA(tA, tB)
            
            let a2 = ConvexCollider(transform: mt, collider: a)

            let contact = collide(a2, b)
            
            if contact.type != .collide {
                return Contact.outside
            }

            return tB.convert(contact)
        }
    }
    
    private func collide(_ a: ConvexCollider, _ b: ConvexCollider) -> Contact {
        let cA = findContact(a, b)
        let cB = findContact(b, a)
        
        if cA.type == .collide && cB.type == .collide {
            let middle = cA.point.middle(cB.point)
            let penetration = (cA.penetration + cB.penetration) / 2
            let count = (cA.count + cB.count) >> 1

            let normal: FixVec
            if cA.penetration < cB.penetration {
                normal = cA.normal.negative
            } else {
                normal = cB.normal
            }
            return Contact(point: middle, normal: normal, penetration: penetration, count: count, type: .collide)
        } else if cB.type == .collide {
            return cB
        } else if cA.type == .collide {
            return cA.negativeNormal()
        } else {
            return Contact.outside
        }
    }
    
    private func findContact(_ a: ConvexCollider, _ b: ConvexCollider) -> Contact {
        var contact_0 = Contact(point: FixVec.zero, normal: FixVec.zero, penetration: Int64.max, count: 1, type: .outside)
        var contact_1 = Contact(point: FixVec.zero, normal: FixVec.zero, penetration: Int64.max, count: 1, type: .outside)
        
        for i in 0..<b.points.count {
            let vert = b.points[i]
            if !a.isContain(vert) {
                continue
            }
            
            var sv: Int64 = Int64.min
            var nv = FixVec.zero
            
            for j in 0..<a.points.count {
                let n = a.normals[j]
                let p = a.points[j]
                
                let d = vert - p
                let s = n.dotProduct(d)
                
                if s > sv {
                    sv = s
                    nv = n
                }
            }
            
            if sv < contact_1.penetration {
                let newContact = Contact(point: vert, normal: nv, penetration: sv, count: 1, type: .collide)
                
                if newContact.penetration < contact_0.penetration {
                    contact_0 = newContact
                } else {
                    contact_1 = newContact
                }
            }
        }
        
        if contact_1.type == .collide {
            let mid = contact_0.point.middle(contact_1.point)
            return Contact(point: mid, normal: contact_0.normal, penetration: contact_0.penetration, count: 2, type: .collide)
        } else if contact_0.type == .collide {
            return contact_0
        } else {
            return .outside
        }
    }
       
}
