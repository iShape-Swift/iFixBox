//
//  VarBody.swift
//  
//
//  Created by Nail Sharipov on 12.05.2023.
//

import iFixFloat

struct VarBody {
    
    let index: Int // global index in bodies Array
    var velocity: Velocity
    var biasVel: Velocity
    
    var dynManifolds: [Int32] = []
    var statManifolds: [Int32] = []
    
    @inlinable
    init(index: Int, velocity: Velocity) {
        self.index = index
        self.velocity = velocity
        self.biasVel = velocity
    }
    
    @inlinable
    mutating func addStat(manifold: Int) {
        statManifolds.append(Int32(manifold))
    }
    
    @inlinable
    mutating func addDyn(manifold: Int) {
        dynManifolds.append(Int32(manifold))
    }
}

struct VarVelocity {
    
    static let zero = VarVelocity(velocity: .zero, count: 0)
    
    private (set) var velocity: Velocity
    private (set) var count: Int

    @inlinable
    mutating func add(velocity v: Velocity) {
        velocity = velocity + v
        count += 1
    }
    
    @inlinable
    func average(impactStabilization: Int64) -> Velocity {
        if count == 1 {
            return velocity
        } else {
            let k = FixFloat.unit / Int64(count) - impactStabilization // every impact body will lose energy
            let linear = k * FixVec(velocity.linear.x, velocity.linear.y)
            let angular = velocity.angular.fixMul(k)
            
            return Velocity(linear: linear, angular: angular)
        }
    }
}
