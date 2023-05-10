//
//  BodyStore.swift
//  
//
//  Created by Nail Sharipov on 09.05.2023.
//

struct BodyStore {

    private var timeStamp: Int

    private var ids: [Int64]
    
    var bodies: [Body]

    @inlinable
    init(capacity: Int) {
        ids = [Int64](repeating: -1, count: capacity)
        bodies = [Body](repeating: .empty, count: capacity)
        timeStamp = 0
    }

    @inlinable
    func actor(handler: BodyHandler) -> Actor {
        let index: Int
        if handler.timeStamp == timeStamp {
            index = handler.index
        } else {
            index = ids.findIndex(value: handler.id)
        }

        let newHandler = BodyHandler(id: handler.id, index: index, timeStamp: timeStamp)

        return Actor(handler: newHandler, body: bodies[index])
    }

    @inlinable
    mutating func set(actor: Actor) -> BodyHandler {
        let index: Int
        if actor.handler.timeStamp == timeStamp {
            index = actor.handler.index
        } else {
            index = ids.findIndex(value: actor.handler.id)
        }

        bodies[index] = actor.body

        return BodyHandler(id: actor.handler.id, index: index, timeStamp: timeStamp)
    }
    
    @inlinable
    mutating func add(body: Body) -> BodyHandler {
        guard !ids.isEmpty else {
            ids.append(body.id)
            bodies.append(body)
            return BodyHandler(id: body.id, index: 0, timeStamp: timeStamp)
        }

        let index = ids.findFreeIndex(value: body.id)
        assert(index != -1, "Index should not be -1")

        if index != ids.count {
            timeStamp += 1
        }

        ids.insert(body.id, at: index)
        bodies.insert(body, at: index)
        
        return BodyHandler(id: body.id, index: index, timeStamp: timeStamp)
    }

    @inlinable
    mutating func remove(handler: BodyHandler) {
        let index: Int
        if handler.timeStamp == timeStamp {
            index = handler.index
        } else {
            index = ids.findIndex(value: handler.id)
        }
        
        assert(index != -1, "Index should not be -1")

        timeStamp += 1
        
        ids.remove(at: index)
        bodies.remove(at: index)
    }

    @inlinable
    mutating func removeAll() {
        ids.removeAll()
        bodies.removeAll()
        timeStamp += 1
    }
}


private extension Array where Element == Int64 {
    
    func findFreeIndex(value: Int64) -> Int {
        var left = 0
        var right = self.count - 1
        var j = -1
        var i = right >> 1
        var x = self[i]

        while i != j {
            if x > value {
                right = i - 1
            } else if x < value {
                left = i + 1
            } else {
                return -1
            }

            j = i
            i = (left + right) >> 1
            x = self[i]
        }

        if x < value {
            i += 1
        }

        return i
    }
    
    func findIndex(value: Int64) -> Int {
        var left = 0
        var right = self.count - 1
        var j = -1
        var i = right >> 1
        var x = self[i]

        while i != j {
            if x > value {
                right = i - 1
            } else if x < value {
                left = i + 1
            } else {
                return i
            }

            j = i
            i = (left + right) >> 1
            x = self[i]
        }

        return -1
    }
    
}
