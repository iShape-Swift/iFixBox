//
//  BodyHandler.swift
//  
//
//  Created by Nail Sharipov on 09.05.2023.
//

public struct BodyHandler {
    
    public static let empty = BodyHandler(id: -1, index: -1, timeStamp: -1)
    
    public let id: Int64
    public let index: Int
    let timeStamp: Int

    public init(id: Int64) {
        self.id = id
        index = -1
        timeStamp = -1
    }
    
    init(id: Int64, index: Int, timeStamp: Int) {
        self.id = id
        self.index = index
        self.timeStamp = timeStamp
    }

}
