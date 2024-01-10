//
//  Size.swift
//  
//
//  Created by Nail Sharipov on 09.05.2023.
//

import iFixFloat

public typealias Size = FixVec

public extension Size {
    
    @inlinable
    var width: FixFloat { x }

    @inlinable
    var height: FixFloat { y }
    
    @inlinable
    var area: FixFloat { x.fixMul(y) }
    
}
