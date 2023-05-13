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
    
    var dynManifolds: [Int32] = []
    var statManifolds: [Int32] = []
    
    init(index: Int, velocity: Velocity) {
        self.index = index
        self.velocity = velocity
    }
    
    mutating func addStat(manifold: Int) {
        statManifolds.append(Int32(manifold))
    }
    
    mutating func addDyn(manifold: Int) {
        dynManifolds.append(Int32(manifold))
    }
}
