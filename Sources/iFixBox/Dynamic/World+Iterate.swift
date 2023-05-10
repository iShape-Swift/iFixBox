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
        let partners: [Int32]
        let vSet: [Bool]
    }
    
    mutating func iterate() {

        var bodies = bodyStore.bodies
        
        var manifolds = prepareManifolds(bodies: bodies)
        
        let varBundle = prepareVars(manifolds: &manifolds, count: bodies.count)
        
        var vars = varBundle.vars
        let vSet = varBundle.vSet
        let partners = varBundle.partners
        
        // resolve (can be iterated multiple times)
        iterateVars(vars: &vars, manifolds: manifolds)

        // integrate no impacts bodies
        integrateNoImpact(bodies: &bodies, vSet: vSet)
        
        // integrate var bodies
        integrateVars(bodies: &bodies, vars: vars, partners: partners)
        
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
                            manifolds.append(Manifold(a: a, b: b, iA: iA, iB: iB, contact: contact))
                        } else {
                            // static will be always B
                            let contact = collisionSolver.collide(b, a)
                            manifolds.append(Manifold(a: b, b: a, iA: iB, iB: iA, contact: contact))
                        }
                    }
                }
            }
        }
        
        return manifolds
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
                    vList.append(VarBody(index: m.iA, velocity: m.a.startVel))
                } else {
                    let vA = vMap[m.iA]
                    var v = vList[vA]
                    v.partnerCount += 1
                    vList[vA] = v
                }
            }
            var vB: Int = -1
            if m.b.isDynamic {
                vSet[m.iA] = true
                vB = vMap[m.iB]
                if vB < 0 {
                    vB = vList.count
                    vMap[m.iB] = vB
                    vList.append(VarBody(index: m.iB, velocity: m.b.startVel))
                } else {
                    let vB = vMap[m.iB]
                    var v = vList[vB]
                    v.partnerCount += 1
                    vList[vB] = v
                }
            }
            m.set(vA: vA, vB: vB)
            manifolds[i] = m
        }

        var capacity: Int32 = 0
        for i in 0..<vList.count {
            var v = vList[i]
            if v.partnerCount > 4 {
                v.data_0 = capacity
                v.data_1 = 0 // use as local counter
                capacity += v.partnerCount
            }
        }

        var partners = [Int32](repeating: -1, count: Int(capacity))
        for m in manifolds {
            if m.vA >= 0 {
                var v = vList[m.vA]
                if v.partnerCount > 4 {
                    let index = Int(v.data_0 + v.data_1)
                    partners[index] = Int32(m.vB)
                } else {
                    v.add(index: m.vB)
                }
                v.data_1 += 1
                vList[m.vA] = v
            }
            
            if m.vB >= 0 {
                var v = vList[m.vB]
                if v.partnerCount > 4 {
                    let index = Int(v.data_0 + v.data_1)
                    partners[index] = Int32(m.vA)
                } else {
                    v.add(index: m.vA)
                }
                v.data_1 += 1
                vList[m.vB] = v
            }
        }
        
        return VarBundle(vars: vList, partners: partners, vSet: vSet)
    }
    
    private func iterateVars(vars: inout [VarBody], manifolds: [Manifold]) {
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
    
    private func integrateNoImpact(bodies: inout [Body], vSet: [Bool]) {
        for i in 0..<bodies.count where !vSet[i] {
            var body = bodies[i]
            
            let acceleration = body.applyGravity ? body.acceleration.linear + gravity : body.acceleration.linear
            
            let v = body.velocity.linear + acceleration * timeStep
            let w = body.velocity.angular + body.acceleration.angular.mul(timeStep)
            
            let p = body.transform.position + v * timeStep
            let a = body.transform.angle + w.mul(timeStep)

            body.stepUpdate(velocity: Velocity(linear: v, angular: w), transform: Transform(position: p, angle: a))
            
            bodies[i] = body
        }
    }
    
    private func integrateVars(bodies: inout [Body], vars: [VarBody], partners: [Int32]) {
        for i in 0..<vars.count {
            let varBody = vars[i]
            var body = bodies[varBody.index]
            
            let p = body.transform.position + varBody.velocity.linear * timeStep
            let a = body.transform.angle + varBody.velocity.angular.mul(timeStep)

            let acceleration = body.applyGravity ? body.acceleration.linear + gravity : body.acceleration.linear
            
            let v = varBody.velocity.linear + acceleration * timeStep
            let w = varBody.velocity.angular + body.acceleration.angular.mul(timeStep)
            
            body.stepUpdate(velocity: Velocity(linear: v, angular: w), transform: Transform(position: p, angle: a))
            
            bodies[varBody.index] = body
        }
    }

}
