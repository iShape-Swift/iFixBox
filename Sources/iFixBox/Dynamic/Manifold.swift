//
//  Manifold.swift
//  
//
//  Created by Nail Sharipov on 09.05.2023.
//

import iFixFloat

struct Other {
    let isA: Bool
    let index: Int
}

struct ImpactSolution {
    
    static let noImpact = ImpactSolution(velA: .zero, velB: .zero, isImpact: false)

    let velA: Velocity
    let velB: Velocity
    let isImpact: Bool
}

struct VarBody {
    
    let index: Int // global index in bodies Array
    var velocity: Velocity
    
    var manifolds: [Int32]
    
    init(index: Int, manifold: Int, velocity: Velocity) {
        self.index = index
        self.velocity = velocity
        manifolds = [Int32(manifold)]
    }
    
    mutating func add(manifold: Int) {
        manifolds.append(Int32(manifold))
    }
}

struct LetBody {

    let transform: Transform
    let startVel: Velocity
    let invMass: FixFloat
    let invInertia: FixFloat
    var isDynamic: Bool { invMass != 0 }
    
    init(body: Body) {
        self.transform = body.transform
        self.startVel = body.velocity
        self.invMass = body.invMass
        self.invInertia = body.invInertia
    }
}


struct Manifold {
    
    let a: LetBody
    let b: LetBody
    let e: FixFloat
    let q: FixFloat
    let contact: Contact
    let iA: Int         // global index in bodies Array
    let iB: Int         // global index in bodies Array
    var vA: Int = -1    // index in vars
    var vB: Int = -1    // index in vars
    
    init(a: Body, b: Body, iA: Int, iB: Int, contact: Contact) {
        self.a = LetBody(body: a)
        self.b = LetBody(body: b)
        self.iA = iA
        self.iB = iB
        self.contact = contact
        self.e = max(a.material.bounce, b.material.bounce)
        self.q = (a.material.friction + b.material.friction) >> 1
    }
    
    mutating func set(vA: Int, vB: Int) {
        self.vA = vA
        self.vB = vB
    }
    
    func resolve(varA: VarBody, varB: VarBody) -> ImpactSolution {
        // normal to contact point
        let n = contact.normal
        
        // start linear and angular velocity for A and B
        let aV1 = varA.velocity.linear
        let aW1 = varA.velocity.angular

        let bV1 = varB.velocity.linear
        let bW1 = varB.velocity.angular
        
        // distance between center of Mass A to contact point
        let aR = contact.point - a.transform.position
        
        // distance between center of Mass B to contact point
        let bR = contact.point - b.transform.position

        // relative velocity
        let rV1 = aV1 - bV1 + aR.crossProduct(aW1) - bR.crossProduct(bW1)

        let rV1proj = rV1.dotProduct(n)
        
        // only if getting closer
        guard rV1proj < 0 else {
            // still can penetrate
            return .noImpact
        }

        // -(1 + e)
        let ke = -e - .unit
        
        // normal impulse
        // -(1 + e) * rV1 * n / (1 / Ma + 1 / Mb + (aR * t)^2 / aI)

        let aRn = aR.crossProduct(n)
        let bRn = bR.crossProduct(n)
        let iNum = rV1proj.mul(ke)
        let iDen = a.invMass + b.invMass + aRn.sqr.mul(a.invInertia) + bRn.sqr.mul(b.invInertia)
        let i = iNum.div(iDen)

        // new linear velocity
        var aV2 = aV1 + i.mul(a.invMass) * n
        var bV2 = bV1 - i.mul(b.invMass) * n
        
        // new angular velocity
        var aW2 = aW1 + aRn.mul(i).mul(a.invInertia)
        var bW2 = bW1 - bRn.mul(i).mul(b.invInertia)

        // tangent vector
        // leaving only the component that is parallel to the contact surface
        var f = rV1 - n * rV1proj
        let sqrF = f.sqrLength

        // ignore if it to small
        if sqrF >= 1 {
            f = f.normalize
            
            let aRf = aR.crossProduct(f)
            let bRf = bR.crossProduct(f)
            let jNum = rV1.dotProduct(f).mul(ke)
            let jDen = a.invMass + b.invMass + aRf.sqr.mul(a.invInertia) + bRf.sqr.mul(b.invInertia)
            var j = jNum.div(jDen)
            
            // can not be more then original impulse
            let maxFi = i.mul(q)
            j = j.clamp(min: -maxFi, max: maxFi)
            
            // new linear velocity
            aV2 = aV2 + j.mul(a.invMass) * f
            bV2 = bV2 - j.mul(b.invMass) * f
        
            // new angular velocity
            aW2 = aW2 + aRf.mul(j).mul(a.invInertia)
            bW2 = bW2 - bRf.mul(j).mul(b.invInertia)
        }

        return ImpactSolution(
            velA: Velocity(linear: aV2, angular: aW2),
            velB: Velocity(linear: bV2, angular: bW2),
            isImpact: true
        )
    }

    
    // TODO cache some parameters
    func resolve(varA: VarBody) -> ImpactSolution {
        // Normal is always look at A * <-| * B
        let n = contact.normal
        
        // start linear and angular velocity for A and B
        let aV1 = varA.velocity.linear
        let aW1 = varA.velocity.angular

        let bV1 = b.startVel.linear
        let bW1 = b.startVel.angular
        
        // distance between center of Mass A to contact point
        let aR = contact.point - a.transform.position
        
        // distance between center of Mass A to contact point
        let bR = contact.point - b.transform.position
        
        // relative velocity
        let rV1 = aV1 - bV1 + aR.crossProduct(aW1) - bR.crossProduct(bW1)

        let rV1proj = rV1.dotProduct(n)
        
        // only if getting closer
        guard rV1proj < 0 else {
            return .noImpact
        }

        // -(1 + e)
        let ke = -e - .unit
        
        // normal impulse
        // -(1 + e) * rV1 * n / (1 / Ma + (aR * t)^2 / aI)
        
        let aRn = aR.crossProduct(n) // can be cached
        let iNum = rV1proj.mul(ke)
        let iDen = a.invMass + aRn.sqr.mul(a.invInertia)
        let i = iNum.div(iDen)

        // new linear velocity
        var aV2 = aV1 + i.mul(a.invMass) * n
        
        // new angular velocity
        var aW2 = aW1 + aRn.mul(i).mul(a.invInertia)

        // tangent vector
        // leaving only the component that is parallel to the contact surface
        var f = rV1 - n * rV1proj
        let sqrF = f.sqrLength

        // ignore if it to small
        if sqrF >= 1 {
            f = f.normalize
            
            let aRf = aR.crossProduct(f)
            let jNum = rV1.dotProduct(f).mul(ke)
            let jDen = a.invMass + aRf.sqr.mul(a.invInertia)
            var j = jNum.div(jDen)
            
            // can not be more then original impulse
            let maxFi = i.mul(q)
            j = j.clamp(min: -maxFi, max: maxFi)
            
            // new linear velocity
            aV2 = aV2 + j.mul(a.invMass) * f
        
            // new angular velocity
            aW2 = aW2 + aRf.mul(j).mul(a.invInertia)
        }

        return ImpactSolution(velA: Velocity(linear: aV2, angular: aW2), velB: .zero, isImpact: true)
    }
  
    func other(index: Int) -> Other {
        if index == vA {
            return Other(isA: true, index: vB)
        } else {
            return Other(isA: false, index: vA)
        }
    }

    func possibleVelocity(varA: Velocity, varB: Velocity) -> FixFloat {
        // normal to contact point
        let n = contact.normal

        let aV1 = varA.linear
        let aW1 = varA.angular

        let aR = contact.point - a.transform.position

        let av = aV1 + aR.crossProduct(aW1)
        let an = av.dotProduct(n)
        
//        if an >= 0 {
//            return .zero
//        }

        let bV1 = varB.linear
        let bW1 = varB.angular
        
        // distance between center of Mass B to contact point
        let bR = contact.point - b.transform.position

        let bv = bV1 + bR.crossProduct(bW1)
        
        let bn = bv.dotProduct(n)
        
        let rn = an - bn
        
        if rn >= -20 {
            return .unit
        }

        if an <= bn {
            return 0
        }

        if an == .zero {
            return 0
        }

        return bn.div(an)
    }
    
    func possibleVelocity(varA: Velocity) -> FixFloat {
        return self.possibleVelocity(varA: varA, varB: b.startVel)
    }
    
}
