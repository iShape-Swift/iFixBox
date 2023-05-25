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
        for _ in 0..<positionIterations {
            self.posIterate()
        }
    }
    
    mutating private func posIterate() {
        var bodies = bodyStore.bodies
        
        var statMans: [StManifold]
        var dynMans: [DmManifold]
        
        if isDebug {
            let manifoldBundle = prepareManifoldsDebug(bodies: bodies)
            statMans = manifoldBundle.statMans
            dynMans = manifoldBundle.dynMans
            contacts = manifoldBundle.contacts
        } else {
            let manifoldBundle = prepareManifolds(bodies: bodies)
            statMans = manifoldBundle.statMans
            dynMans = manifoldBundle.dynMans
        }
        
        let varBundle = prepareVars(dynMans: &dynMans, statMans: &statMans, bodies: bodies)
        
        var vars = varBundle.vars
        let vSet = varBundle.vSet
        
        if !statMans.isEmpty || !dynMans.isEmpty {
            // resolve (can be iterated multiple times)
            iterateVars(vars: &vars, dynMans: dynMans, statMans: statMans)
        }

        // integrate no impacts bodies
        integrateNoImpact(bodies: &bodies, vSet: vSet)
        
        // integrate var bodies
        integrateVarsClassic(bodies: &bodies, vars: vars)
        
        bodyStore.bodies = bodies
    }
    
    
    private func prepareManifolds(bodies: [Body]) -> (dynMans: [DmManifold], statMans: [StManifold]) {
        
        var dynMans = [DmManifold]()
        var statMans = [StManifold]()
        
        let capacity = 16 + (bodies.count >> 16)
        
        dynMans.reserveCapacity(capacity)
        statMans.reserveCapacity(capacity)
        
        // find all contacts
        
        // can be parallelized
        for iA in 0..<bodies.count - 1 {
            let a = bodies[iA]
            for iB in iA + 1..<bodies.count {
                let b = bodies[iB]
                
                if a.isDynamic || b.isDynamic {
                    if a.boundary.isCollide(b.boundary) {
                        
                        if a.isDynamic && b.isDynamic {
                            let contact = collisionSolver.collide(a, b)
                            if contact.status != .outside {
                                dynMans.append(DmManifold(a: a, b: b, iA: iA, iB: iB, contact: contact, iTimeStep: iTimeStep))
                            }
                        } else if a.isDynamic {
                            let contact = collisionSolver.collide(a, b)
                            if contact.status != .outside {
                                statMans.append(StManifold(a: a, b: b, iA: iA, iB: iB, contact: contact, iTimeStep: iTimeStep))
                            }
                        } else {
                            let contact = collisionSolver.collide(b, a)
                            if contact.status != .outside {
                                statMans.append(StManifold(a: b, b: a, iA: iB, iB: iA, contact: contact, iTimeStep: iTimeStep))
                            }
                        }
                    }
                }
            }
        }
        
        return (dynMans, statMans)
    }
    
    private func prepareManifoldsDebug(bodies: [Body]) -> (dynMans: [DmManifold], statMans: [StManifold], contacts: [Contact]) {

        var dynMans = [DmManifold]()
        var statMans = [StManifold]()
        var contacts = [Contact]()
        
        let capacity = 16 + (bodies.count >> 16)
        
        dynMans.reserveCapacity(capacity)
        statMans.reserveCapacity(capacity)
        
        // find all contacts
        
        // can be parallelized
        for iA in 0..<bodies.count - 1 {
            let a = bodies[iA]
            for iB in iA + 1..<bodies.count {
                let b = bodies[iB]
                
                if a.isDynamic || b.isDynamic {
                    if a.boundary.isCollide(b.boundary) {
                        
                        if a.isDynamic && b.isDynamic {
                            let contact = collisionSolver.collide(a, b)
                            if contact.status != .outside {
                                dynMans.append(DmManifold(a: a, b: b, iA: iA, iB: iB, contact: contact, iTimeStep: iTimeStep))
                                contacts.append(contact)
                            }
                        } else if a.isDynamic {
                            let contact = collisionSolver.collide(a, b)
                            if contact.status != .outside {
                                statMans.append(StManifold(a: a, b: b, iA: iA, iB: iB, contact: contact, iTimeStep: iTimeStep))
                                contacts.append(contact)
                            }
                        } else {
                            let contact = collisionSolver.collide(b, a)
                            if contact.status != .outside {
                                statMans.append(StManifold(a: b, b: a, iA: iB, iB: iA, contact: contact, iTimeStep: iTimeStep))
                                contacts.append(contact)
                            }
                        }
                    }
                }
            }
        }
        
        return (dynMans, statMans, contacts)
    }

    private func prepareVars(dynMans: inout [DmManifold], statMans: inout [StManifold], bodies: [Body]) -> VarBundle {
        
        let count = bodies.count
        var vSet = [Bool](repeating: false, count: count)
        
        var vList = [VarBody]()
        vList.reserveCapacity(2 * dynMans.count + statMans.count)
        
        var vMap = [Int](repeating: -1, count: count)
        vMap.reserveCapacity(vList.count)
        
        for i in 0..<dynMans.count {
            var m = dynMans[i]

            vSet[m.iA] = true
            vSet[m.iB] = true

            var vA = vMap[m.iA]
            if vA < 0 {
                let a = bodies[m.iA]
                vA = vList.count
                vMap[m.iA] = vA
                var v = VarBody(index: m.iA, velocity: a.velocity)
                v.addDyn(manifold: i)
                vList.append(v)
            } else {
                let vA = vMap[m.iA]
                var v = vList[vA]
                v.addDyn(manifold: i)
                vList[vA] = v
            }
            
            var vB: Int = -1
            
            
            vB = vMap[m.iB]
            if vB < 0 {
                let b = bodies[m.iB]
                vB = vList.count
                vMap[m.iB] = vB
                var v = VarBody(index: m.iB, velocity: b.velocity)
                v.addDyn(manifold: i)
                vList.append(v)
            } else {
                let vB = vMap[m.iB]
                var v = vList[vB]
                v.addDyn(manifold: i)
                vList[vB] = v
            }
            
            m.vA = vA
            m.vB = vB
            
            dynMans[i] = m
        }
        
        for i in 0..<statMans.count {
            var m = statMans[i]

            vSet[m.iA] = true
            
            var vA = vMap[m.iA]
            if vA < 0 {
                let a = bodies[m.iA]
                vA = vList.count
                vMap[m.iA] = vA
                var v = VarBody(index: m.iA, velocity: a.velocity)
                v.addDyn(manifold: i)
                vList.append(v)
            } else {
                let vA = vMap[m.iA]
                var v = vList[vA]
                v.addStat(manifold: i)
                vList[vA] = v
            }
            
            m.vA = vA
            
            statMans[i] = m
        }
        
        return VarBundle(vars: vList, vSet: vSet)
    }
    
    private func iterateVars(vars: inout [VarBody], dynMans: [DmManifold], statMans: [StManifold]) {
        for _ in 0..<velocityIterations {
            for m in dynMans {
                var vA = vars[m.vA]
                var vB = vars[m.vB]
                let solution = m.resolve(varA: vA, varB: vB)
                if solution.isImpact {
                    vA.velocity = solution.velA
                    vB.velocity = solution.velB
                    vars[m.vA] = vA
                    vars[m.vB] = vB
                }
            }
            
            for m in statMans {
                var vA = vars[m.vA]
                let solution = m.resolve(varA: vA)
                if solution.isImpact {
                    vA.velocity = solution.vel
                    vars[m.vA] = vA
                }
            }
        }
    }
    
    private func integrateNoImpact(bodies: inout [Body], vSet: [Bool]) {
        for i in 0..<bodies.count where !vSet[i] {
            var body = bodies[i]
            
            guard (!body.velocity.isZero || body.applyGravity) && body.isAlive else { continue }
            
            let acceleration = body.applyGravity ? body.acceleration.linear + gravity : body.acceleration.linear
            
            let v = body.velocity.linear + acceleration * posTimeStep
            let w = body.velocity.angular + body.acceleration.angular.mul(posTimeStep)
            
            let p = body.transform.position + v * posTimeStep
            let a = body.transform.angle + w.mul(posTimeStep)

            let velocity = Velocity(linear: v, angular: w)
            let transform = Transform(position: p, angle: a)
            let boundary = self.boundary(shape: body.shape, transform: transform)
            
            body.stepUpdate(velocity: velocity, transform: transform, boundary: boundary)
            
            bodies[i] = body
        }
    }
    
    private func integrateVarsClassic(bodies: inout [Body], vars: [VarBody]) {
        for i in 0..<vars.count {
            let varBody = vars[i]

            var body = bodies[varBody.index]
            let acceleration = body.applyGravity ? body.acceleration.linear + gravity : body.acceleration.linear
            
            let v = varBody.velocity.linear + acceleration * posTimeStep
            let w = varBody.velocity.angular + body.acceleration.angular.mul(posTimeStep)
            
            let p = body.transform.position + v * posTimeStep
            let a = body.transform.angle + w.mul(posTimeStep)

            let velocity = Velocity(linear: v, angular: w)
            let transform = Transform(position: p, angle: a)
            let boundary = self.boundary(shape: body.shape, transform: transform)
            
            body.stepUpdate(velocity: velocity, transform: transform, boundary: boundary)
            
            bodies[varBody.index] = body
        }
    }

}
