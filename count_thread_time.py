f=open('output_further_parallel.txt', 'r')

lines = f.readlines()

sum_time = 0
count = 0

for line in lines:
    if 'EstimateTranslationThread' in line:
        sum_time += int(line.strip().split(' ')[3])
        count += 1

print("EstimateTranslationThread average time is: ", float(sum_time)/count)