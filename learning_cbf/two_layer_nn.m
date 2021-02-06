function [output] = two_layer_nn(input,w1,b1,w2,b2)

output = b2 + (w2 * max(0, (w1 * input + b1)));

end

