def test():
    global x
    x = 1
    y = 2
    return y

x = 2
y = test()
print(x,y)

z = -6%10
print(z)