library(ggplot2)
library(reshape2)

df <- read.csv("~/workspace/sssp-fpga/data/timing.csv")

# absolute timing graphs per dataset
for(datasetName in levels(df$dataset)) {
  print(datasetName)
  subdf <- df[df$dataset == datasetName,]
  melted <- melt(subdf, id.vars = c('version', 'device'), measure.vars = 'time')
  
  ggplot(melted, aes(version, value)) + 
    geom_bar(aes(fill = device), position = 'dodge', stat = 'identity') +
    xlab('Algorithm version') +
    ylab('Time to complete, ms') +
    ggtitle(datasetName)
  
  
  ggsave(paste(datasetName, '.png'), plot = last_plot(), width=10, height=5)
}