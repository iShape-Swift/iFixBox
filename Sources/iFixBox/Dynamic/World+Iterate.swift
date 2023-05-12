//
//  World+Iterate.swift
//  
//
//  Created by Nail Sharipov on 09.05.2023.
//

import iFixFloat

public extension World {
    
    struct VarBundle {
        let vars: [VarBody]
        let vSet: [Bool]
    }
    
    mutating func iterate() {

        var bodies = bodyStore.bodies
        
        var manifolds: [Manifold]
        
        if isDebug {
            let manifoldBundle = prepareManifoldsDebug(bodies: bodies)
            manifolds = manifoldBundle.manifolds
            contacts = manifoldBundle.contacts
        } else {
            manifolds = prepareManifolds(bodies: bodies)
        }
        
        let varBundle = prepareVars(manifolds: &manifolds, count: bodies.count)
        
        var vars = varBundle.vars
        let vSet = varBundle.vSet
        
        // resolve (can be iterated multiple times)
        iterateVars(vars: &vars, manifolds: manifolds)

        // integrate no impacts bodies
        integrateNoImpact(bodies: &bodies, vSet: vSet)
        
        // integrate var bodies
        integrateVarsClassic(bodies: &bodies, manifolds: manifolds, vars: vars)
        
        bodyStore.bodies = bodies
    }
    
    private func prepareManifolds(bodies: [Body]) -> [Manifold] {

        var manifolds = [Manifold]()
        manifolds.reserveCapacity(16 + (bodies.count >> 16))
        
        // find all contacts
        
        // can be parallelized
        for iA in 0..<bodies.count - 1 {
            let a = bodies[iA]
            for iB in iA + 1..<bodies.count {
                let b = bodies[iB]
                
                if a.isDynamic || b.isDynamic {
                    if a.boundary.isCollide(b.boundary) {
                        if a.isDynamic {
                            let contact = collisionSolver.collide(a, b)
                            if contact.type != .outside {
                                manifolds.append(Manifold(a: a, b: b, iA: iA, iB: iB, contact: contact, iTimeStep: iTimeStep))
                            }
                        } else {
                            // static will be always B
                            let contact = collisionSolver.collide(b, a)
                            if contact.type != .outside {
                                manifolds.append(Manifold(a: b, b: a, iA: iB, iB: iA, contact: contact, iTimeStep: iTimeStep))
                            }
                        }
                    }
                }
            }
        }
        
        return manifolds
    }
    
    private func prepareManifoldsDebug(bodies: [Body]) -> (manifolds: [Manifold], contacts: [Contact]) {

        var manifolds = [Manifold]()
        var contacts = [Contact]()
        manifolds.reserveCapacity(16 + (bodies.count >> 16))
        
        // find all contacts
        
        // can be parallelized
        for iA in 0..<bodies.count - 1 {
            let a = bodies[iA]
            for iB in iA + 1..<bodies.count {
                let b = bodies[iB]
                
                if a.isDynamic || b.isDynamic {
                    if a.boundary.isCollide(b.boundary) {
                        if a.isDynamic {
                            let contact = collisionSolver.collide(a, b)
                            if contact.type != .outside {
                                manifolds.append(Manifold(a: a, b: b, iA: iA, iB: iB, contact: contact, iTimeStep: iTimeStep))
                                contacts.append(contact)
                            }
                        } else {
                            // static will be always B
                            let contact = collisionSolver.collide(b, a)
                            if contact.type != .outside {
                                manifolds.append(Manifold(a: b, b: a, iA: iB, iB: iA, contact: contact, iTimeStep: iTimeStep))
                                contacts.append(contact)
                            }
                        }
                    }
                }
            }
        }
        
        return (manifolds, contacts)
    }

    private func prepareVars(manifolds: inout [Manifold], count: Int) -> VarBundle {
        
        var vSet = [Bool](repeating: false, count: count)
        
        var vList = [VarBody]()
        vList.reserveCapacity(2 * manifolds.count)
        
        var vMap = [Int](repeating: -1, count: count)
        vMap.reserveCapacity(vList.count)
        
        for i in 0..<manifolds.count {
            var m = manifolds[i]
            var vA: Int = -1
            if m.a.isDynamic {
                vSet[m.iA] = true
                vA = vMap[m.iA]
                if vA < 0 {
                    vA = vList.count
                    vMap[m.iA] = vA
                    vList.append(VarBody(index: m.iA, manifold: i, velocity: m.a.startVel))
                } else {
                    let vA = vMap[m.iA]
                    var v = vList[vA]
                    v.add(manifold: i)
                    vList[vA] = v
                }
            }
            var vB: Int = -1
            if m.b.isDynamic {
                vSet[m.iB] = true
                vB = vMap[m.iB]
                if vB < 0 {
                    vB = vList.count
                    vMap[m.iB] = vB
                    vList.append(VarBody(index: m.iB, manifold: i, velocity: m.b.startVel))
                } else {
                    let vB = vMap[m.iB]
                    var v = vList[vB]
                    v.add(manifold: i)
                    vList[vB] = v
                }
            }
            m.set(vA: vA, vB: vB)
            manifolds[i] = m
        }
        
        return VarBundle(vars: vList, vSet: vSet)
    }
    
    private func iterateVars(vars: inout [VarBody], manifolds: [Manifold]) {
        for _ in 0..<10 {
            for m in manifolds {
                
                // A can not be static
                if m.vB >= 0 {
                    var vA = vars[m.vA]
                    var vB = vars[m.vB]
                    let solution = m.resolve(varA: vA, varB: vB)
                    if solution.isImpact {
                        vA.velocity = solution.velA
                        vB.velocity = solution.velB
                        vars[m.vA] = vA
                        vars[m.vB] = vB
                    }
                } else {
                    // B is static
                    var vA = vars[m.vA]
                    let solution = m.resolve(varA: vA)
                    if solution.isImpact {
                        vA.velocity = solution.velA
                        vars[m.vA] = vA
                    }
                }
            }
        }
    }
    
    private func integrateNoImpact(bodies: inout [Body], vSet: [Bool]) {
        for i in 0..<bodies.count where !vSet[i] {
            var body = bodies[i]
            
            guard (!body.velocity.isZero || body.applyGravity) && body.isAlive else { continue }
            
            let acceleration = body.applyGravity ? body.acceleration.linear + gravity : body.acceleration.linear
            
            let v = body.velocity.linear + acceleration * timeStep
            let w = body.velocity.angular + body.acceleration.angular.mul(timeStep)
            
            let p = body.transform.position + v * timeStep
            let a = body.transform.angle + w.mul(timeStep)

            body.stepUpdate(velocity: Velocity(linear: v, angular: w), transform: Transform(position: p, angle: a))
            
            bodies[i] = body
        }
    }
    
    private func integrateVarsClassic(bodies: inout [Body], manifolds: [Manifold], vars: [VarBody]) {
        for i in 0..<vars.count {
            let varBody = vars[i]

            var body = bodies[varBody.index]
            let acceleration = body.applyGravity ? body.acceleration.linear + gravity : body.acceleration.linear
            
            let v = varBody.velocity.linear + acceleration * timeStep
            let w = varBody.velocity.angular + body.acceleration.angular.mul(timeStep)
            
            let p = body.transform.position + v * timeStep
            let a = body.transform.angle + w.mul(timeStep)

            body.stepUpdate(velocity: Velocity(linear: v, angular: w), transform: Transform(position: p, angle: a))
            
            bodies[i] = body
        }
    }
    
    
    private func integrateVars(bodies: inout [Body], manifolds: [Manifold], vars: [VarBody]) {
        for i in 0..<vars.count {
            let varBody = vars[i]
            var max: FixFloat = .unit
            for j in varBody.manifolds {
                let m = manifolds[Int(j)]
                let k: FixFloat
                let other = m.other(index: i)
                if other.index >= 0 {
                    let otherBody = vars[other.index]
                    if other.isA {
                        k = m.possibleVelocity(varA: varBody.velocity, varB: otherBody.velocity)
                    } else {
                        k = m.possibleVelocity(varA: otherBody.velocity, varB: varBody.velocity)
                    }
                } else {
                    k = m.possibleVelocity(varA: varBody.velocity)
                }
                
                if k == .zero {
                    max = 0
                    break
                }
                
                max = Swift.min(max, k)
            }
            
            let moveLinear = varBody.velocity.linear * max
            let moveAngular = varBody.velocity.angular.mul(max / 2)

            var body = bodies[varBody.index]

            let p = body.transform.position + moveLinear * timeStep
            let a = body.transform.angle + moveAngular.mul(timeStep)

            let acceleration = body.applyGravity ? body.acceleration.linear + gravity : body.acceleration.linear

            let v = varBody.velocity.linear + acceleration * timeStep
            let w = varBody.velocity.angular + body.acceleration.angular.mul(timeStep)

            body.stepUpdate(velocity: Velocity(linear: v, angular: w), transform: Transform(position: p, angle: a))

            bodies[varBody.index] = body
        }
    }
    

}
