//
//  World+Shape.swift
//  
//
//  Created by Nail Sharipov on 31.05.2023.
//

import iFixFloat

extension World {

    func calcFeature(shape: Shape) -> Feature {
        let area: FixFloat
        let unitInertia: FixFloat
        let center: FixVec

        switch shape.form {
        case .circle:
            let rr = shape.radius.sqr
            area = FixFloat.pi.mul(rr)
            unitInertia = rr / 2
            center = .zero
        case .rect:
            let size = shape.size
            area = size.area
            unitInertia = size.sqrLength.mul(85) // 85 ~= 1024 / 12
            center = .zero
        case .polygon:
            area = 0
            unitInertia = 0
            center = .zero
        }

        return Feature(area: area, unitInertia: unitInertia, center: center)
    }
    
//    func createShape([]) -> Shape {
//        
//    }
//    
//    func attache(shape: Shape, ) {
//        
//    }
}
    
