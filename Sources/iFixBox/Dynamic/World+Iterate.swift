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
        var dynMans = [DmManifold]()
        var statMans = [StManifold]()
        
        let capacity = 16 + (bodies.count >> 16)
        
        dynMans.reserveCapacity(capacity)
        statMans.reserveCapacity(capacity)

        for _ in 0..<positionIterations {
            
            dynMans.removeAll(keepingCapacity: true)
            statMans.removeAll(keepingCapacity: true)
            
            if isDebug {
                contacts.removeAll(keepingCapacity: true)
                prepareManifoldsDebug(bodies: bodies, dynMans: &dynMans, statMans: &statMans, contacts: &contacts)
            } else {
                prepareManifolds(bodies: bodies, dynMans: &dynMans, statMans: &statMans)
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
            integrateVars(bodies: &bodies, vars: vars, dynMans: dynMans, statMans: statMans)

            bodyStore.bodies = bodies
        }
    }
    
    
    private func prepareManifolds(bodies: [Body], dynMans: inout [DmManifold], statMans: inout [StManifold]) {
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
                                dynMans.append(DmManifold(a: a, b: b, iA: iA, iB: iB, contact: contact, biasScale: biasScale))
                            }
                        } else if a.isDynamic {
                            let contact = collisionSolver.collide(a, b)
                            if contact.status != .outside {
                                statMans.append(StManifold(a: a, b: b, iA: iA, iB: iB, contact: contact, biasScale: biasScale))
                            }
                        } else {
                            let contact = collisionSolver.collide(b, a)
                            if contact.status != .outside {
                                statMans.append(StManifold(a: b, b: a, iA: iB, iB: iA, contact: contact, biasScale: biasScale))
                            }
                        }
                    }
                }
            }
        }
    }
    
    private func prepareManifoldsDebug(bodies: [Body], dynMans: inout [DmManifold], statMans: inout [StManifold], contacts: inout [Contact]) {
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
                                dynMans.append(DmManifold(a: a, b: b, iA: iA, iB: iB, contact: contact, biasScale: biasScale))
                                contacts.append(contact)
                            }
                        } else if a.isDynamic {
                            let contact = collisionSolver.collide(a, b)
                            if contact.status != .outside {
                                statMans.append(StManifold(a: a, b: b, iA: iA, iB: iB, contact: contact, biasScale: biasScale))
                                contacts.append(contact)
                            }
                        } else {
                            let contact = collisionSolver.collide(b, a)
                            if contact.status != .outside {
                                statMans.append(StManifold(a: b, b: a, iA: iB, iB: iA, contact: contact, biasScale: biasScale))
                                contacts.append(contact)
                            }
                        }
                    }
                }
            }
        }
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
        let stabilization = self.impactStabilization
        
        let dirtVels = UnsafeMutablePointer<VarVelocity>.allocate(capacity: vars.count)
        dirtVels.initialize(repeating: .zero, count: vars.count)

        let biasVels = UnsafeMutablePointer<VarVelocity>.allocate(capacity: vars.count)
        biasVels.initialize(repeating: .zero, count: vars.count)
        
        vars.withUnsafeMutableBufferPointer { vars in
            var vel = VarVelocity.zero
            
            for _ in 0..<velocityIterations {
                
                for m in dynMans {
                    let vA = vars[m.vA]
                    let vB = vars[m.vB]
                    
                    let solution = m.resolve(velA: vA.velocity, velB: vB.velocity)
                    if solution.isImpact {
                        vel = dirtVels[m.vA]
                        vel.add(velocity: solution.velA)
                        dirtVels[m.vA] = vel
                        
                        vel = dirtVels[m.vB]
                        vel.add(velocity: solution.velB)
                        dirtVels[m.vB] = vel
                    }
                    
                    let biasSol = m.resolveBias(velA: vA.biasVel, velB: vB.biasVel)
                    if biasSol.isImpact {
                        vel = biasVels[m.vA]
                        vel.add(velocity: biasSol.velA)
                        biasVels[m.vA] = vel
                        
                        vel = biasVels[m.vB]
                        vel.add(velocity: biasSol.velB)
                        biasVels[m.vB] = vel
                    }
                }
                
                for m in statMans {
                    let vA = vars[m.vA]
                    let solution = m.resolve(velA: vA.velocity)
                    if solution.isImpact {
                        vel = dirtVels[m.vA]
                        vel.add(velocity: solution.vel)
                        dirtVels[m.vA] = vel
                    }
                    
                    let biasSol = m.resolveBias(velA: vA.biasVel)
                    if biasSol.isImpact {
                        vel = biasVels[m.vA]
                        vel.add(velocity: biasSol.vel)
                        biasVels[m.vA] = vel
                    }
                }
                
                var anyImpact = false
                
                for i in 0..<vars.count {
                    var v = vars[i]
                    let dirtVel = dirtVels[i]
                    let biasVel = biasVels[i]
                    
                    if biasVel.count > 0 || dirtVel.count > 0 {
                        if dirtVel.count > 0 {
                            v.velocity = dirtVel.average(impactStabilization: stabilization)
                            dirtVels[i] = .zero
                        }
                        
                        if biasVel.count > 0 {
                            v.biasVel = biasVel.average(impactStabilization: stabilization)
                            biasVels[i] = .zero
                        }
                        
                        vars[i] = v
                        
                        anyImpact = true
                    }
                }
                
                if !anyImpact {
                    return
                }
            }
        }
        
        biasVels.deallocate()
        dirtVels.deallocate()
    }
    
    private func integrateNoImpact(bodies: inout [Body], vSet: [Bool]) {
        for i in 0..<bodies.count where !vSet[i] {
            var body = bodies[i]
            
            guard (!body.velocity.isZero || body.applyGravity) && body.isAlive else { continue }
            
            let acceleration = body.applyGravity ? body.acceleration.linear + gravity : body.acceleration.linear
            
            let v = body.velocity.linear + acceleration * posTimeStep
            let w = body.velocity.angular + body.acceleration.angular.fixMul(posTimeStep)
            
            let p = body.transform.position + v * posTimeStep
            let a = body.transform.angle + w.fixMul(posTimeStep)

            let velocity = Velocity(linear: v, angular: w)
            let transform = Transform(position: p, angle: a)
            let boundary = self.boundary(shape: body.shape, transform: transform)
            
            body.stepUpdate(velocity: velocity, transform: transform, boundary: boundary)
            
            bodies[i] = body
        }
    }
    
    private func integrateVars(bodies: inout [Body], vars: [VarBody], dynMans: [DmManifold], statMans: [StManifold]) {
        for i in 0..<vars.count {
            let v = vars[i]

            var body = bodies[v.index]
            let acceleration = body.applyGravity ? body.acceleration.linear + gravity : body.acceleration.linear

            // update position, we use bias impact here
            
            let pos = body.transform.position + v.biasVel.linear * posTimeStep
            let ang = body.transform.angle + v.biasVel.angular.fixMul(posTimeStep)
            
            let transform = Transform(position: pos, angle: ang)
            let boundary = self.boundary(shape: body.shape, transform: transform)
            
            // update velocity, do not include bias here
            
            let lin = v.velocity.linear + acceleration * posTimeStep
            let rot = v.velocity.angular + body.acceleration.angular.fixMul(posTimeStep)
            
            let velocity = Velocity(linear: lin, angular: rot)
            
            body.stepUpdate(velocity: velocity, transform: transform, boundary: boundary)

            bodies[v.index] = body
        }
    }

}
