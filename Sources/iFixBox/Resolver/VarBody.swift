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
    
    init(index: Int, velocity: Velocity) {
        self.index = index
        self.velocity = velocity
        self.biasVel = velocity
    }
    
    mutating func addStat(manifold: Int) {
        statManifolds.append(Int32(manifold))
    }
    
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
    var average: Velocity {
        if count == 1 {
            return velocity
        } else {
            let n = Int64(count)
            let linear = FixVec(velocity.linear.x / n, velocity.linear.y / n)
            let angular = velocity.angular / n
            
            return Velocity(linear: linear, angular: angular)
        }
    }
}
