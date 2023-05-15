//
//  CollisionSolver+ConvexToConvex.swift
//  
//
//  Created by Nail Sharipov on 09.05.2023.
//

import iFixFloat

extension CollisionSolver {

    public func collide(_ a: ConvexCollider, _ b: ConvexCollider, tA: Transform, tB: Transform) -> Contact {
        if a.points.count > b.points.count {
            let mt = Transform.convertFromBtoA(tB, tA)
            
            let b2 = ConvexCollider(transform: mt, collider: b)

            let contact = collide(a, b2)

            return tA.convert(contact)
        } else {
            let mt = Transform.convertFromBtoA(tA, tB)
            
            let a2 = ConvexCollider(transform: mt, collider: a)

            let contact = collide(a2, b)
            
            if contact.status != .collide {
                return Contact.outside
            }

            return tB.convert(contact)
        }
    }
    
    private func collide(_ a: ConvexCollider, _ b: ConvexCollider) -> Contact {
        let cA = findContact(a, b)
        let cB = findContact(b, a)
        
        if cA.status == .collide && cB.status == .collide {
            if cA.type == cB.type {
                if cA.penetration < cB.penetration {
                    return cA.negativeNormal()
                } else {
                    return cB
                }
            } else if cA.type == .edge {
                return cA.negativeNormal()
            } else {
                return cB
            }
        } else if cB.status == .collide {
            return cB
        } else if cA.status == .collide {
            return cA.negativeNormal()
        } else {
            return Contact.outside
        }
    }
    
    private func findContact(_ a: ConvexCollider, _ b: ConvexCollider) -> Contact {
        var c0_nm = FixVec.zero
        var c0_pt = FixVec.zero
        var c0_vi = -1
        var c0_pn: FixFloat = Int64.max
        

        var c1_pt = FixVec.zero
        var c1_vi = -1
        var c1_pn: FixFloat = Int64.max
        
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
            
            if sv < c1_pn {
                if sv < c0_pn {
                    c0_pt = vert
                    c0_nm = nv
                    c0_pn = sv
                    c0_vi = i
                } else {
                    c1_pt = vert
                    c1_pn = sv
                    c1_vi = i
                }
            }
        }
        
        if c1_vi >= 0 {
            let n = b.points.count
            
            let m = (c0_pt + c1_pt).half
            
            var sv: Int64 = Int64.min
            var nv = FixVec.zero
            
            for j in 0..<a.points.count {
                let n = a.normals[j]
                let p = a.points[j]
                
                let d = m - p
                let s = n.dotProduct(d)
                
                if s > sv {
                    sv = s
                    nv = n
                }
            }

            let type: ContactType
            
            if (c0_vi + 1) % n == c1_vi || (c1_vi + 1) % n == c0_vi  {
                type = .edge
            } else {
                type = .average
            }
            
            return Contact(point: m, normal: nv, penetration: sv, status: .collide, type: type)
            
        } else if c0_vi >= 0 {
            return Contact(point: c0_pt, normal: c0_nm, penetration: c0_pn, status: .collide, type: .vertex)
        } else {
            return .outside
        }
    }
       
}
